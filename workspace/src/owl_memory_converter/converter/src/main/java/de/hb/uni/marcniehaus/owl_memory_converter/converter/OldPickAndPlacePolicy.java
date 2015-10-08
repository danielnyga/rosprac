package de.hb.uni.marcniehaus.owl_memory_converter.converter;

import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.LogElement;
import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.OWLLogElement;
import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.Task;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
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
    public List<String> getErrors(Task t)  throws Exception {
        //TODO: Should errors be thrown in child tasks?!
        LinkedList<String> toReturn = new LinkedList<>();
        Collection<LogElement> failures = t.getOtherObjectProperties().
                get("eventFailure");
        if(failures==null)
            return toReturn;
        for(LogElement failure : failures) {
            if(!(failure instanceof OWLLogElement))
                throw new Exception("Only errors from OWL are supported!");
            OWLLogElement error = (OWLLogElement) failure;
            toReturn.add(error.getOwlClassName());
        }
        return toReturn;
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
