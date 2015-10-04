package de.hb.uni.marcniehaus.owl_memory_converter.converter;

import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.Task;
import java.util.LinkedList;
import java.util.List;
import robot_memory.Object;

public class OldPickAndPlacePolicy implements 
        OwlConverter.TaskTreeConversionPolicy {
    
    public OldPickAndPlacePolicy(OwlConverter.MessageSender messageFactory) {
        mMessageFactory = messageFactory;
    }

    @Override
    public boolean isApplicable(Task rootTask) {
        return true; //TODO
    }

    @Override
    public List<String> getErrors(Task t) {
        return new LinkedList<>(); //TODO
    }

    @Override
    public String getGoal(Task t) {
        return ""; //TDOO
    }

    @Override
    public List<Object> getObjectsActedOn(Task t) {
        return new LinkedList<>(); //TODO
    }

    @Override
    public List<Object> getPerceivedObjects(Task t) {
        return new LinkedList<>(); //TODO
    }

    @Override
    public boolean skip(Task t) {
        return "WITH-FAILURE-HANDLING".equals(t.getContext()) || 
               "WITH-DESIGNATORS".equals(t.getContext());
    }
    
    private OwlConverter.MessageSender mMessageFactory;
}
