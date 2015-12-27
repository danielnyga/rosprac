package de.hb.uni.marcniehaus.owl_memory_converter.converter;

import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.LogElement;
import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.MongoLogElement;
import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.OWLLogElement;
import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.Task;

import java.util.*;

import robot_memory.Object;
import robot_memory.Tuple;

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
    public String getErrors(Task t)  throws Exception {
        //TODO: Should errors be thrown in child tasks?!
        String toReturn = null;
        Collection<LogElement> failures = t.getOtherObjectProperties().
                get("eventFailure");
        if(failures==null) {
            return toReturn;
        }
        for(LogElement failure : failures) {
            if(!(failure instanceof OWLLogElement))
                throw new Exception("Only errors from OWL are supported!");
            OWLLogElement error = (OWLLogElement) failure;
            if(toReturn==null) {
                toReturn=error.getOwlClassName();
            } else {
                throw new Exception("Only one error at a time is supported!");
            }
        }
        return toReturn;
    }

    @Override
    public List<Tuple> getParameters(Task t) throws Exception {
        List<Tuple> toReturn = new LinkedList<>();
        Map<String, Collection<String>> parameters = getPropertiesFromTask(t);
        parameters.putAll(getPropertiesFromActionDesignator(t));
        for(Map.Entry<String, Collection<String>> entry : parameters.entrySet()) {
            if(entry.getKey().equals("OWLName") ||
                    entry.getKey().equals("taskContext") ||
                    entry.getKey().equals("OWLClassName") ||
                    entry.getKey().startsWith("_")) {
                continue;
            }
            if(entry.getValue().size()!=1) {
                throw new Exception("Only properties with size 1 are supported!");
            }
            Tuple tuple = mMessageFactory.createMessage(Tuple._TYPE);
            tuple.setName(entry.getKey());
            tuple.setValue(entry.getValue().iterator().next());
            toReturn.add(tuple);
        }
        return toReturn;
    }

    @Override
    public List<Object> getUsedObjects(Task t) throws Exception {
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

    protected Map<String, Collection<String>> getPropertiesFromTask(Task t) throws Exception {
        return t.getOtherDataProperties();
    }

    protected Map<String, Collection<String>> getPropertiesFromActionDesignator(Task t) throws Exception {
        Map<String, Collection<String>> dataProperties = getDataPropertiesOfChildObjectProperty(
                t, 3, new Predicate<LogElement>() {
            @Override
            public boolean test(LogElement element) {
                Map<String, Collection<String>> dataProperties = element.getOtherDataProperties();
                return  dataProperties.containsKey("_designator_type") &&
                        dataProperties.get("_designator_type").size()==1 &&
                        dataProperties.get("_designator_type").iterator().next().toLowerCase().equals("action")
                        ||
                        dataProperties.containsKey("_DESIGNATOR_TYPE") &&
                        dataProperties.get("_DESIGNATOR_TYPE").size()==1 &&
                        dataProperties.get("_DESIGNATOR_TYPE").iterator().next().toLowerCase().equals("action");
            }
        });
        return dataProperties;
    }

    private interface Predicate<T> {
        boolean test(T element);
    }

    private Map<String, Collection<String>> getDataPropertiesOfChildObjectProperty(
            LogElement parent, int recursionDepth, Predicate<LogElement> dataPropertyTest) {
        Map<String, Collection<String>> toReturn = new HashMap<>();
        if(recursionDepth>0) {
            for(Collection<LogElement> childCollection : parent.getOtherObjectProperties().values()) {
                for(LogElement child : childCollection) {
                    toReturn.putAll(getDataPropertiesOfChildObjectProperty(child, recursionDepth-1, dataPropertyTest));
                }
            }
        } else {
            if(dataPropertyTest.test(parent)) {
                return parent.getOtherDataProperties();
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
        LinkedList<Tuple> properties = new LinkedList<>();
        o.setProperties(properties);
        for(Map.Entry<String, Collection<String>> propertyGroup : 
                designator.getOtherDataProperties().entrySet()) {
            if(propertyGroup.getKey().toLowerCase().equals(("name"))) {
                for(String property : propertyGroup.getValue()) {
                    o.setObjectId(property);
                }
            } else if(propertyGroup.getKey().toLowerCase().equals("_id")
                    && o.getObjectId().equals("")) {
                for(String property : propertyGroup.getValue()) {
                    o.setObjectId(property);
                }
            } else if(propertyGroup.getKey().toLowerCase().equals("type")) {
                for(String property : propertyGroup.getValue()) {
                    o.setObjectType(property);
                }
            } else if(!propertyGroup.getKey().startsWith("_") && 
                    !excludedPropertyKeys.contains(propertyGroup.getKey())){
                for(String propertyValue : propertyGroup.getValue()) {
                    if(!excludedPropertyValues.contains(propertyValue)) {
                        Tuple prop = mMessageFactory.
                                createMessage(Tuple._TYPE);
                        properties.add(prop);
                        prop.setName(propertyGroup.getKey());
                        prop.setValue(propertyValue);
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