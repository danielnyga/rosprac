package de.hb.uni.marcniehaus.owl_memory_converter.converter;

import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.LogElement;
import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.OWLLogElement;
import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.Task;

import java.util.*;

public class RobohowPickAndPlacePolicy extends OldPickAndPlacePolicy {

    private final String ANNOTATED_PARAMETER_TYPE = "annotatedParameterType";
    private final String ARM = "ARM";

    public RobohowPickAndPlacePolicy(OwlConverter.MessageSender messageFactory) {
        super(messageFactory);
    }

    @Override
    public boolean isApplicable(Task rootTask) {
        Stack<LogElement> elementsToLookAt = new Stack<>();
        HashSet<LogElement> elementThatHaveBeenSeen = new HashSet<>();
        elementsToLookAt.add(rootTask);
        while(!elementsToLookAt.isEmpty()) {
            LogElement current = elementsToLookAt.pop();
            elementThatHaveBeenSeen.add(current);
            if(current.getOtherDataProperties().containsKey(ANNOTATED_PARAMETER_TYPE)) {
                return true;
            }
            for(Collection<LogElement> childCollection : current.getOtherObjectProperties().values()) {
                for(LogElement child : childCollection) {
                    if(!elementThatHaveBeenSeen.contains(child)) {
                        elementsToLookAt.push(child);
                    }
                }
            }
        }
        return false;
    }

    @Override
    protected Map<String, Collection<String>> getPropertiesFromTask(Task t) throws Exception {
        Map<String, Collection<String>> toReturn = new HashMap<>();
        for(Map.Entry<String, Collection<String>> entry : super.getPropertiesFromTask(t).entrySet()) {
            if (entry.getKey().equals(ARM)) {
                String elements = "";
                for(String value : entry.getValue()) {
                    elements = elements + ", " + value;
                }
                List<String> toAdd = new ArrayList<>();
                toAdd.add(elements.substring(0, elements.length()-1));
                toReturn.put(entry.getKey(), toAdd);
            }else if(!entry.getKey().equals(ANNOTATED_PARAMETER_TYPE)) {
                toReturn.put(entry.getKey(), entry.getValue());
            }

        }
        return toReturn;
    }
}
