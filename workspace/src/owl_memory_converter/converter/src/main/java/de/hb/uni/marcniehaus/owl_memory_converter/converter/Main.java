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
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.concurrent.Semaphore;

import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.Task;
import org.ros.exception.RemoteException;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.internal.message.Message;
import org.ros.internal.node.topic.SubscriberIdentifier;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.DefaultPublisherListener;
import org.ros.node.topic.Publisher;
import robot_memory.LearningTriggerRequest;
import robot_memory.LearningTriggerResponse;
import robot_memory.RobotState;
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
            if(args.length<1 || !new File(args[args.length-1]).exists()) {
                System.err.println(
                        "Usage: rosrun owl_memory_converter converter owlfile");
                return;
            }
            NodeMainExecutor exec = DefaultNodeMainExecutor.newDefault();
            String owlFileName = args[args.length-1];
            ArrayList<String> extendedArgs = Lists.newArrayList(args);
            extendedArgs.remove(extendedArgs.size()-1);
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
        try{
            initCommunication(connectedNode);
            OwlConverter converter = new OwlConverter(this);
            for(Task rootTask : converter.getRootTasks(mOwlFileName)) {
                try {
                    mNumberOfPublishedStates = 0;
                    TriggerResponse tresponse = mStartTrainingTrigger.call(
                            mStartTrainingTrigger.newMessage());
                    SynchronousService.throwErrorIfTriggerFailed(
                            tresponse.getSuccess(), tresponse.getMessage());
                    converter.convert(rootTask);
                    LearningTriggerRequest request = mLearnTrigger.newMessage();
                    connectedNode.getLog().info("Waiting for learner to finish...");
                    request.setNumberOfRequiredMessages(mNumberOfPublishedStates);
                    LearningTriggerResponse lresponse = mLearnTrigger.call(request);
                    SynchronousService.throwErrorIfTriggerFailed(
                            lresponse.getSuccess(), lresponse.getMessage());
                } catch (Exception e) {
                    connectedNode.getLog().error("Caught Exception: " + e.getMessage());
                    e.printStackTrace(System.err);
                }
            }
        } catch(Exception e){
            connectedNode.getLog().error("Caught Exception: " + e.getMessage());
            e.printStackTrace(System.err);
        } finally {
            connectedNode.getLog().info("Finished! Closing down...");
            closeCommunication();
        }
    }
    
    private void closeCommunication() {      
        System.exit(0);
    }
    
    private void initCommunication(ConnectedNode node) throws Exception {
        ServiceClient<TriggerRequest,TriggerResponse> tmpTrigger = 
                node.newServiceClient(
                "robot_memory/start_collecting_training_data", 
                std_srvs.Trigger._TYPE);
        mStartTrainingTrigger = new SynchronousService<>(tmpTrigger);
        ServiceClient<LearningTriggerRequest,LearningTriggerResponse> 
                tmpLearningTrigger = node.newServiceClient("robot_memory/learn", 
                robot_memory.LearningTrigger._TYPE);
        mLearnTrigger = new SynchronousService<>(tmpLearningTrigger);        
        mStatePublisher = node.newPublisher(
                "robot_memory/state", RobotState._TYPE);
        final Semaphore wait = new Semaphore(0);
        node.getLog().info("Waiting for subscriber...");
        mStatePublisher.addListener(new DefaultPublisherListener<RobotState>(){
            @Override
            public void onNewSubscriber(Publisher<RobotState> publisher, 
                    SubscriberIdentifier subscriberIdentifier) {
                wait.release();
            }            
        });
        wait.acquire();
        node.getLog().info("subscriber registered!");
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
    
    private static class SynchronousService<Request, Response>{
        
        public SynchronousService(ServiceClient<Request,Response> client) {
            mTrigger = client;
        }
        
        public Request newMessage() {
            return mTrigger.newMessage();
        }
        
        public Response call(Request request) throws Exception {
            mTrigger.call(request,
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
                
        public static void throwErrorIfTriggerFailed
                (boolean success, String message) throws Exception{
            if(!success) {
                throw new Exception(message);
            }
        }
        
        private final ServiceClient<Request,Response> mTrigger;
        private final Semaphore mSemaphore = new Semaphore(0);
        private Exception mException;
        private Response mResponse;
    }
         
    private Publisher<RobotState> mStatePublisher;
    private SynchronousService
            <TriggerRequest, TriggerResponse> mStartTrainingTrigger;
    private SynchronousService 
            <LearningTriggerRequest, LearningTriggerResponse> mLearnTrigger;
    private long mNumberOfPublishedStates;
    private final String mOwlFileName;
    private NodeConfiguration mConfig;
}
