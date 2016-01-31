package de.hb.uni.marcniehaus.owl_memory_converter.converter;

import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.LogElement;
import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.OWLExtractor;
import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.OWLLogElement;
import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.Task;
import java.io.File;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import org.ros.internal.message.Message;
import org.ros.message.Time;
import robot_memory.RobotState;

public class OwlConverter {    
    
    public OwlConverter(MessageSender sender) {
        mPublisher = sender;
        mConversionPolicies = 
            new TaskTreeConversionPolicy[]{
                new RobohowPickAndPlacePolicy(sender),
                new OldPickAndPlacePolicy(sender)
            };
    }
    
    public interface MessageSender {
        public <T extends Message>T createMessage(String name) throws Exception;
        public void send(RobotState state) throws Exception;
    }
    
    public interface TaskTreeConversionPolicy extends 
                                PreAndPostOrderTaskIterator.IteratorCallback {
        public boolean isApplicable(Task rootTask) throws Exception;
        public String getErrors(Task t) throws Exception;
        public List<robot_memory.Tuple> getParameters(Task t) throws Exception;
        public List<robot_memory.Object> getUsedObjects(Task t)
                throws Exception;
        public List<robot_memory.Object> getPerceivedObjects(Task t) 
                throws Exception;
        
    }

    public Collection<Task> getRootTasks(String owlFileName) throws Exception {
        System.out.println("Extract from file " + owlFileName);
        OWLExtractor ex = OWLExtractor.crateExtractor(
                new File(owlFileName), new HashSet<String>());
        return ex.getRootTasks();
    }

    public void convert(Task rootTask) throws Exception {
        TaskTreeConversionPolicy policy = getFirstFittingPolicy(rootTask);
        int sequenceNumber = 0;
        for(PreAndPostOrderTaskIterator iterator = 
                new PreAndPostOrderTaskIterator(rootTask, policy);
                iterator.hasNext(); sequenceNumber++) {
            iterator.next();   
            RobotState s = getState(iterator, sequenceNumber, policy);
            mPublisher.send(s);
        }
    }

    private RobotState getState(PreAndPostOrderTaskIterator iterator,
                                int sequenceNumber,
                                TaskTreeConversionPolicy conversionPolicy)
            throws Exception {
        RobotState toReturn = mPublisher.createMessage(RobotState._TYPE);
        toReturn.setFinished(iterator.currentHasBeenVisitedBefore());
        toReturn.setTaskId(iterator.current().getOwlInstanceName());
        toReturn.setSequenceNumber(sequenceNumber);
        toReturn.setTaskName(iterator.current().getContext());
        String error = conversionPolicy.getErrors(iterator.current());
        if(error!=null) {
            toReturn.setError(error);
        }
        toReturn.setParameters(conversionPolicy.getParameters(iterator.current()));
        toReturn.setUsedObjects(
                conversionPolicy.getUsedObjects(iterator.current()));
        toReturn.setPerceivedObjects(
                conversionPolicy.getPerceivedObjects(iterator.current()));

        String timeProperty = iterator.currentHasBeenVisitedBefore()
                ? "endTime" : "startTime";
        Collection<LogElement> timeObjects = iterator.current().
                getOtherObjectProperties().get(timeProperty);
        if(timeObjects==null || timeObjects.size()!=1)
            throw new Exception("No start or end time found!");
        LogElement timeObject = timeObjects.iterator().next();
        if(!(timeObject instanceof OWLLogElement))
            throw new Exception("The time object must be received by OWL");
        String timeString = ((OWLLogElement) timeObject).getOwlInstanceName();
        int unixTime;
        if(timeString.matches("timepoint_[0-9]+")) {
            unixTime = Integer.parseInt(timeString.split("_")[1]);
        } else if(timeString.matches("timepoint_[0-9]+.[0-9]+")) {
            unixTime = Integer.parseInt(timeString.split("_")[1].split("\\.")[0]);
        } else {
            throw new Exception("Wrong time format: " + timeString + "!");
        }
        toReturn.setCurrentTime(new Time(unixTime, 0));
        return toReturn;
    }
    
    private TaskTreeConversionPolicy getFirstFittingPolicy(Task rootTask) 
                                                            throws Exception{
        for(TaskTreeConversionPolicy p : mConversionPolicies) {
            if(p.isApplicable(rootTask))
                return p;
        }
        throw new Exception("No applicable policy found!");
    }
    
    private final MessageSender mPublisher;
    private final TaskTreeConversionPolicy[] mConversionPolicies;
}