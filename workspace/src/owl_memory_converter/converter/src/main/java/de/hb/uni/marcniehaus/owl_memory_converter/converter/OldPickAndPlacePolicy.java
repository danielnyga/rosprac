package de.hb.uni.marcniehaus.owl_memory_converter.converter;

import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.LogElement;
import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.MongoLogElement;
import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.OWLLogElement;
import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.Task;
import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Stack;
import robot_memory.Object;
import robot_memory.ObjectProperty;

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
        if(failures==null) {
            return toReturn;
        }
        for(LogElement failure : failures) {
            if(!(failure instanceof OWLLogElement))
                throw new Exception("Only errors from OWL are supported!");
            OWLLogElement error = (OWLLogElement) failure;
            toReturn.add(error.getOwlClassName());
        }
        return toReturn;
    }

    @Override
    public String getGoal(Task t) throws Exception {
        Collection<String> goal = t.getOtherDataProperties().get("goalContext");
        if(goal==null) {
            return "";
        }
        if(goal.size()>1) {
            throw new Exception("Only one goal is supported!");
        }
        return goal.iterator().next();
    }

    @Override
    public List<Object> getObjectsActedOn(Task t) throws Exception {
        List<Object> toReturn = new LinkedList<>();
        Collection<LogElement> action = 
                t.getOtherObjectProperties().get("objectActedOn");
        if(action==null) {
            return toReturn;
        }
        String location = getLocation(t);
        for(LogElement resultDesignators : action) {
            Collection<LogElement> subDesignators = resultDesignators.
                    getOtherObjectProperties().get("designator");
            Collection<Collection<LogElement>> groups;
            if(subDesignators!=null) {
                groups = new LinkedList<>();
                for(LogElement subDesignator : subDesignators) {
                    groups.addAll(subDesignator.
                            getOtherObjectProperties().values());
                }
            } else {
                groups = resultDesignators.getOtherObjectProperties().values();
            }
            for(Collection<LogElement> group : groups) {
                for(LogElement designator : group) {
                    if(designator instanceof MongoLogElement) {
                        toReturn.addAll(getObjects(
                                (MongoLogElement) designator, location));
                    }
                }
            }
        }
        return toReturn;    
    }

    @Override
    public List<Object> getPerceivedObjects(Task t) throws Exception {
        List<Object> toReturn = new LinkedList<>();
        Collection<LogElement> perception = 
                t.getOtherObjectProperties().get("perceptionResult");
        if(perception==null) {
            return toReturn;
        }
        String location = getLocation(t);
        for(LogElement resultDesignators : perception) {
            for(Collection<LogElement> group : 
                    resultDesignators.getOtherObjectProperties().values()) {
                for(LogElement designator : group) {
                    if(designator instanceof MongoLogElement) {
                        toReturn.addAll(getObjects(
                                (MongoLogElement) designator, location));
                    }
                }
            }
        }
        return toReturn;
    }
    
    private String getLocation(Task t) {
        Stack<LogElement> elements = new Stack<>();
        elements.add(t);        
        while(!elements.empty()) {
            LogElement current = elements.pop();
            Collection<String> designatorType 
                    = current.getOtherDataProperties().get("_DESIGNATOR_TYPE");
            if(designatorType==null) {
                designatorType = current.getOtherDataProperties().
                        get("_designator_type");
            }
            if(designatorType!=null) {
                if(designatorType.contains("LOCATION") || 
                        designatorType.contains("location")) {
                    Collection<String> names = 
                            current.getOtherDataProperties().get("NAME");
                    if(names!=null) {
                        for(String name : names) {
                            return name;
                        }
                    }
                }
            }            
            for(Map.Entry<String, Collection<LogElement>> childGroup : 
                    current.getOtherObjectProperties().entrySet()) {
                if(childGroup.getKey().equals("successorDesignator")) {
                    continue;
                }
                for(LogElement child : childGroup.getValue()) {
                    elements.add(child);
                }
            }
        }
        return null;
    }
    
    private List<Object> getObjects(MongoLogElement mongoDbDesignator, 
            String location) throws Exception {
        LinkedList<Object> toReturn = new LinkedList<>();
        Collection<LogElement> designators = mongoDbDesignator.
                getOtherObjectProperties().get("designator");
        if(designators==null) {
            return toReturn;
        }
        for(LogElement designator : designators) {
            Object o = getObjectProperties(designator, 
                    getExcludedObjectPropertyKeys(), 
                    getExcludedObjectPropertyValues());
            if(location!=null) {
                o.setObjectLocation(location);
            }
            toReturn.add(o);
        }
        return toReturn;
    }       
    
    private Set<String> getExcludedObjectPropertyKeys() {
        Set<String> toReturn = new HashSet<>();
        toReturn.add("Z-OFFSET");
        return toReturn;                
    }
    
    private Set<String> getExcludedObjectPropertyValues() {
        Set<String> toReturn = new HashSet<>();
        toReturn.add("UNKNOWN");
        return toReturn;        
    }

    private Object getObjectProperties(
            LogElement designator, Set<String> excludedPropertyKeys, 
            Set<String> excludedPropertyValues) throws Exception {
        Object o = mMessageFactory.createMessage(Object._TYPE);
        LinkedList<ObjectProperty> properties = new LinkedList<>();
        o.setProperties(properties);
        for(Map.Entry<String, Collection<String>> propertyGroup : 
                designator.getOtherDataProperties().entrySet()) {
            if(propertyGroup.getKey().equals(("NAME"))) {
                for(String property : propertyGroup.getValue()) {
                    o.setObjectId(property);
                }
            } else if(propertyGroup.getKey().equals("TYPE")) {
                for(String property : propertyGroup.getValue()) {
                    o.setObjectType(property);
                }
            } else if(!propertyGroup.getKey().startsWith("_") && 
                    !excludedPropertyKeys.contains(propertyGroup.getKey())){
                for(String propertyValue : propertyGroup.getValue()) {
                    if(!excludedPropertyValues.contains(propertyValue)) {
                        ObjectProperty prop = mMessageFactory.
                                createMessage(ObjectProperty._TYPE);
                        properties.add(prop);
                        prop.setPropertyName(propertyGroup.getKey());
                        prop.setPropertyValue(propertyValue);   
                    }                       
                }
            }
        }
        return o;
    }
    
    @Override
    public boolean skip(Task t) {
        return "WITH-FAILURE-HANDLING".equals(t.getContext()) || 
               "WITH-DESIGNATORS".equals(t.getContext());
    }
    
    private final OwlConverter.MessageSender mMessageFactory;
}

/*
TODO:
- Place Failure one level higher:
    * The event failure is currently thrown in ON-PROCESS-MODULE
    * The event failure should be better thrown in UIMA-PERCEIVE
- Identify and skip unnecessary functions
    * Some nodes (e.g. GOAL-PERFORM) seem to be unnecessary
    * Distinguish difference between GOAL-ACHIEVE and GOAL-PERFORM 
        Idea: GOAL-ACHIEVE tells CRAM to achieve a goal
              GOAL-PERFORM is the actual execution of the goal function after
                           prerequesites have been fulfilledd
- Add currently unused information:
    * PerceptionRequest
    * ActionDesignators
*/