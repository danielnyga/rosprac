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
                new OldPickAndPlacePolicy(sender)
            };
    }
    
    public interface MessageSender {
        public <T extends Message>T createMessage(String name) throws Exception;
        public void send(RobotState state) throws Exception;
    }
    
    public interface TaskTreeConversionPolicy extends 
                                PreAndPostOrderTaskIterator.IteratorCallback {
        public boolean isApplicable(Task rootTask);
        public List<String> getErrors(Task t);
        public String getGoal(Task t);
        public List<robot_memory.Object> getObjectsActedOn(Task t);
        public List<robot_memory.Object> getPerceivedObjects(Task t);
        
    }
    
    public void convert(String owlFileName) throws Exception {
        System.out.println("Extract from file " + owlFileName);
        OWLExtractor ex = OWLExtractor.crateExtractor(
                new File(owlFileName), new HashSet<String>());
        Collection<Task> tasks = ex.getRootTasks();
        if(tasks.size()!=1)
            throw new Exception("Only one root task is supported!");
        Task rootTask = tasks.iterator().next();
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
        toReturn.setErrors(conversionPolicy.getErrors(iterator.current()));
        toReturn.setGoal(conversionPolicy.getGoal(iterator.current()));
        toReturn.setObjectsActedOn(
                conversionPolicy.getObjectsActedOn(iterator.current()));
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
        if(!timeString.matches("timepoint_[0-9]+"))
            throw new Exception("Wrong time format!");
        int unixTime = Integer.parseInt(timeString.split("_")[1]);
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