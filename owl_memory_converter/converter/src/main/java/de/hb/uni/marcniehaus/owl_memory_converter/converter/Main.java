/*
 * Copyright (C) 2014 Marc Niehaus.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package de.hb.uni.marcniehaus.owl_memory_converter.converter;

import com.google.common.collect.Lists;
import java.io.File;
import java.util.ArrayList;
import java.util.concurrent.Semaphore;
import java.util.concurrent.ThreadFactory;

import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.Task;
import org.ros.exception.RemoteException;
import org.ros.exception.ServiceException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.internal.message.Message;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Publisher;
import owl_memory_converter.Conversion;
import owl_memory_converter.ConversionRequest;
import owl_memory_converter.ConversionResponse;
import task_tree_msgs.LearningTrigger;
import task_tree_msgs.LearningTriggerRequest;
import task_tree_msgs.LearningTriggerResponse;
import task_tree_msgs.RobotState;
import std_srvs.TriggerRequest;
import std_srvs.TriggerResponse;

public class Main extends AbstractNodeMain 
    implements OwlConverter.MessageSender{

    /***
     * A main that is improved in contrast to the RosRun main:
     * - No arguments required
     * @param args 
     */
    public static void main(String[] args) {
        try {
            String owlFileName = null;
            ArrayList<String> extendedArgs = Lists.newArrayList(args);
            if(args.length<1 || !new File(args[args.length-1]).exists()) {
                System.out.println("Starting OWL memory converter as service...");
            } else {
                owlFileName = args[args.length-1];
                extendedArgs.remove(extendedArgs.size()-1);
            }
            NodeMainExecutor exec = DefaultNodeMainExecutor.newDefault();
            extendedArgs.add(Main.class.getClass().getName());
            CommandLineLoader loader = new CommandLineLoader(extendedArgs);   
            NodeConfiguration config = loader.build();
            Main conv = new Main(owlFileName, config);
            exec.execute(conv, config);
        } catch(Exception e) {
            e.printStackTrace(System.err);
        }        
    }
    
    public Main(String owlFileName, NodeConfiguration config) {
        mOwlFileName = owlFileName;
        mConfig = config;
    }
    
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("owl_memory_converter");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        mNode = connectedNode;
        try{
            initCommunication(connectedNode);
        } catch(Exception e){
            connectedNode.getLog().error("Caught Exception: " + e.getMessage());
            e.printStackTrace(System.err);
            closeCommunication();
        }
        if(mOwlFileName!=null) {
            try {
                startConversion(mOwlFileName, true);
            } catch(Exception e) {
                connectedNode.getLog().error("Caught Exception: " + e.getMessage());
                e.printStackTrace(System.err);
            }
            finally {
                connectedNode.getLog().info("Finished! Closing down...");
                closeCommunication();
            }
        } else { //TODO: Avoid or enable multiple requests at a time...
            mNode.newServiceServer(
                    "owl_memory_converter/convert",
                    Conversion._TYPE,
                    new ServiceResponseBuilder<ConversionRequest,
                            ConversionResponse>() {
                        @Override
                        public void build(ConversionRequest request,
                                          ConversionResponse response)
                                throws ServiceException {
                            try {
                                startConversion(request.getOwlFileName(),
					request.getExtendOldModel());
                                response.setSuccess(true);
                            } catch (Exception e) {
                                connectedNode.getLog().error(
                                        "Caught Exception: " + e.getMessage());
                                e.printStackTrace();
                                response.setSuccess(false);
                            }
                        }
                    });
        }
    }

    private void startConversion(String owlFileName, boolean extendOldModel) 
    		throws Exception {
        mNode.getLog().info("Starting conversion of " + owlFileName + "!");
        waitUntilFileHasBeenWritten(owlFileName);
        OwlConverter converter = new OwlConverter(this);
        boolean firstTask = true;
        for(Task rootTask : converter.getRootTasks(owlFileName)) {
            try {
                mNumberOfPublishedStates = 0;
                TriggerResponse tresponse = mStartTrainingTrigger.call(
                        mStartTrainingTrigger.newMessage());
                throwErrorIfTriggerFailed(
                        tresponse.getSuccess(), tresponse.getMessage());
                converter.convert(rootTask);
                LearningTriggerRequest request = mLearnTrigger.newMessage();
                mNode.getLog().info("Waiting for learner to finish...");
                request.setNumberOfRequiredMessages(mNumberOfPublishedStates);
                request.setExtendOldModel(extendOldModel || !firstTask);
                LearningTriggerResponse lresponse = mLearnTrigger.call(request);
                throwErrorIfTriggerFailed(
                        lresponse.getSuccess(), lresponse.getMessage());
            } catch (Exception e) {
                mNode.getLog().error("Caught Exception: " + e.getMessage());
                e.printStackTrace(System.err);
            }
            firstTask = false;
        }
    }

    private void waitUntilFileHasBeenWritten(String filename) throws Exception{
        long newLength = new File(filename).exists() ?
                new File(filename).length() : 0;
        long oldLength;
        do {
            oldLength = newLength;
            mNode.getLog().info("waiting 1s for the owl file...");
            Thread.sleep(1000);
            newLength = new File(filename).exists() ?
                    new File(filename).length() : 0;
        } while(newLength == 0 || newLength > 0 && oldLength != newLength);
    }

    private void closeCommunication() {      
        System.exit(0);
    }
    
    private void initCommunication(ConnectedNode node) throws Exception {
        ServiceClient<TriggerRequest,TriggerResponse> tmpTrigger =
                waitForService("robot_memory/start_collecting_training_data",
                               std_srvs.Trigger._TYPE);
        mStartTrainingTrigger = new SynchronousService<>(tmpTrigger);
        ServiceClient<LearningTriggerRequest,LearningTriggerResponse> 
                tmpLearningTrigger = waitForService("robot_memory/learn",
                LearningTrigger._TYPE);
        mLearnTrigger = new SynchronousService<>(tmpLearningTrigger);        
        mStatePublisher = node.newPublisher(
                "robot_memory/state", RobotState._TYPE);
        while(mStatePublisher.getNumberOfSubscribers()==0) {
            node.getLog().info("Waiting 1s until a subscriber is registered...");
            Thread.sleep(1000);
            node.getLog().info("subscriber registered!");
        }
    }

    @Override
    public <T extends Message> T createMessage(String name) throws Exception {
        return mConfig.getTopicMessageFactory().
                newFromType(name);
    }

    @Override
    public void send(RobotState state) throws Exception {
        mStatePublisher.publish(state);
        mNumberOfPublishedStates++;
        Thread.sleep(1);
        //Bad workaround, but the message queue size is limited and
        //I cannot guarantee transport otherwise...
    }

    @Override
    public void logWarning(String warning) throws Exception {
        mNode.getLog().warn(warning);
    }

    private <T, S> ServiceClient<T, S> waitForService(String serviceName,
                                                      String serviceType)
                                                        throws Exception {
        while(true) {
            try {
                return mNode.newServiceClient(serviceName, serviceType);
            } catch(ServiceNotFoundException e) {
                mNode.getLog().info("Waiting 1s for service " + serviceName);
                Thread.sleep(1000);
            }
        }
    }

    private static void throwErrorIfTriggerFailed
            (boolean success, String message) throws Exception{
        if(!success) {
            throw new Exception(message);
        }
    }

    private static class SynchronousService<Request, Response>{
        
        public SynchronousService(ServiceClient<Request,Response> client) {
            mService = client;
        }
        
        public Request newMessage() {
            return mService.newMessage();
        }
        
        public Response call(Request request) throws Exception {
            mService.call(request,
                    new ServiceResponseListener<Response>() {            
                @Override
                public void onSuccess(Response mt) {
                    mException = null;
                    mResponse = mt;
                    mSemaphore.release();                    
                }
                @Override
                public void onFailure(RemoteException re) {
                    mException = re;
                    mSemaphore.release();
                }
            });
            mSemaphore.acquire();
            if(mException!=null)
                throw mException;
            return mResponse;
        }

        private final ServiceClient<Request,Response> mService;
        private final Semaphore mSemaphore = new Semaphore(0);
        private Exception mException;
        private Response mResponse;
    }

    private ConnectedNode mNode;
    private Publisher<RobotState> mStatePublisher;
    private SynchronousService
            <TriggerRequest, TriggerResponse> mStartTrainingTrigger;
    private SynchronousService 
            <LearningTriggerRequest, LearningTriggerResponse> mLearnTrigger;
    private long mNumberOfPublishedStates;
    private final String mOwlFileName;
    private NodeConfiguration mConfig;
}
