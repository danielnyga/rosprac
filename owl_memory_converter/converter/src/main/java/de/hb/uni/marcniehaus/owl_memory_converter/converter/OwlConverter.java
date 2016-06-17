package de.hb.uni.marcniehaus.owl_memory_converter.converter;

import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.*;

import java.io.File;
import java.util.*;

import org.ros.internal.message.Message;
import org.ros.message.Time;
import task_tree_msgs.Designator;
import task_tree_msgs.RobotState;
import task_tree_msgs.Tuple;

public class OwlConverter {    
    
    public OwlConverter(MessageSender sender) {
        mPublisher = sender;
    }
    
    public interface MessageSender {
        <T extends Message>T createMessage(String name) throws Exception;
        void send(RobotState state) throws Exception;
        void logWarning(String warning) throws Exception;
    }
    
    public Collection<Task> getRootTasks(String owlFileName) throws Exception {
        OWLExtractor ex = OWLExtractor.crateExtractor(
                new File(owlFileName), new HashSet<String>());
        return ex.getRootTasks();
    }

    public void convert(Task rootTask) throws Exception {
        int sequenceNumber = 0;
        for(PreAndPostOrderTaskIterator iterator = 
                new PreAndPostOrderTaskIterator(rootTask);
                iterator.hasNext(); sequenceNumber++) {
            iterator.next();   
            RobotState s = getState(iterator, sequenceNumber);
            mPublisher.send(s);
        }
    }

    private RobotState getState(PreAndPostOrderTaskIterator iterator, int sequenceNumber)
            throws Exception {
        RobotState toReturn = mPublisher.createMessage(RobotState._TYPE);
        toReturn.setFinished(iterator.currentHasBeenVisitedBefore());
        toReturn.setTaskId(iterator.current().getOwlInstanceName());
        toReturn.setSequenceNumber(sequenceNumber);
        toReturn.setCurrentTime(getTime(iterator));
        toReturn.setTaskName(iterator.current().getContext().startsWith(Constants.PREFIX_GOAL_FUNCTION)
                ? iterator.current().getContext().substring(Constants.PREFIX_GOAL_FUNCTION.length())
                : iterator.current().getContext());
        toReturn.setSuccess(successful(iterator.current()));
        setDesignatorsAndGoalContext(iterator.current(), toReturn);
        return toReturn;
    }

    private String getGoalContext(LogElement potentialGoalFunction) {
        if(     !potentialGoalFunction.getOtherObjectProperties().containsKey(Constants.PROPERTY_NAME_PATTERN) ||
                potentialGoalFunction.getOtherObjectProperties().get(Constants.PROPERTY_NAME_PATTERN).size()!=1)
            return "";
        String toReturn = "";
        Stack<LogElement> stack = new Stack<>();
        stack.push(potentialGoalFunction.getOtherObjectProperties().
                get(Constants.PROPERTY_NAME_PATTERN).iterator().next());
        while(!stack.isEmpty()) {
            LogElement current = stack.pop();
            for(Collection<String> list : current.getOtherDataProperties().values()) {
                for(String value : list) {
                    toReturn+=" " + value;
                }
            }
            for(Collection<LogElement> list : current.getOtherObjectProperties().values()) {
                for(LogElement child : list) {
                    stack.push(child);
                }
            }
        }
        return toReturn.trim();
    }

    private Time getTime(PreAndPostOrderTaskIterator iterator) throws Exception {
        String timeProperty = iterator.currentHasBeenVisitedBefore()
                ? Constants.PROPERTY_NAME_END_TIME : Constants.PROPERTY_NAME_START_TIME;
        Collection<LogElement> timeObjects = iterator.current().
                getOtherObjectProperties().get(timeProperty);
        if(timeObjects==null || timeObjects.size()!=1)
            throw new Exception("No start or end time found!");
        LogElement timeObject = timeObjects.iterator().next();
        if(!(timeObject instanceof OWLLogElement))
            throw new Exception("The time object must be received by OWL");
        String timeString = ((OWLLogElement) timeObject).getOwlInstanceName();
        int unixTime;
        if(timeString.matches(Constants.REGEX_TIME_POINT_SECONDS)) {
            unixTime = Integer.parseInt(timeString.split("_")[1]);
        } else if(timeString.matches(Constants.REGEX_TIME_POINT_MILLISECONDS)) {
            unixTime = Integer.parseInt(timeString.split("_")[1].split("\\.")[0]);
        } else if(timeString.matches(Constants.REGEX_TIME_POINT_EMPTY)) { //bug...
	   unixTime = 0;
	} else {
            throw new Exception("Wrong time format: " + timeString + "!");
        }
        return new Time(unixTime, 0);
    }

    private boolean successful(Task t) throws Exception {
        if(t.getOtherDataProperties().containsKey(Constants.PROPERTY_NAME_TASK_SUCCESS)) {
            for(String value : t.getOtherDataProperties().get(Constants.PROPERTY_NAME_TASK_SUCCESS)) {
                if(value.toLowerCase().trim().equals(Constants.PROPERTY_VALUE_TRUE)) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        throw new Exception("Property taskSuccess is not available!");
    }

    private void setDesignatorsAndGoalContext(Task t, RobotState robotStateMessage) throws Exception {
        LinkedList<Tuple> designatorIds = new LinkedList<>();
        LinkedList<Tuple> goalProperties = new LinkedList<>();
        LinkedList<Designator> designators = new LinkedList<>();
        for(Map.Entry<String, Collection<LogElement>> entries : t.getOtherObjectProperties().entrySet()) {
            for(LogElement child : entries.getValue()) {
                if(!(child instanceof OWLDesignator)) {
                    continue;
                }
                for(Collection<LogElement> designatorChildren : child.getOtherObjectProperties().values()) {
                    for(LogElement mongoDbLink : designatorChildren) {
                        if (!(mongoDbLink instanceof MongoLogElement) || !mongoDbLink.
                                getOtherObjectProperties().containsKey(Constants.PROPERTY_NAME_DESIGNATOR)) {
                            continue;
                        }
                        for(LogElement actualDesignator : mongoDbLink.getOtherObjectProperties().
                                get(Constants.PROPERTY_NAME_DESIGNATOR)) {
                            LinkedList<Map.Entry<String, LogElement>> subDesignators = null;
                            robotStateMessage.setGoalContext(getGoalContext(actualDesignator));
                            if(robotStateMessage.getGoalContext().equals("")) {
                                subDesignators = new LinkedList<>();
                                subDesignators.add(new AbstractMap.SimpleEntry<String, LogElement>(
                                        entries.getKey(), actualDesignator));
                            } else {
                                subDesignators = getDesignatorsForGoal(actualDesignator);
                            }
                            for(Map.Entry<String, LogElement> keyValuePair : subDesignators) {
                                Tuple tuple = mPublisher.createMessage(Tuple._TYPE);
                                tuple.setName(keyValuePair.getKey());
                                if(isPose(keyValuePair.getValue())) {
                                    tuple.setValue(poseToString(keyValuePair.getValue()));
                                    goalProperties.add(tuple);
                                } else if(isPoseStamped(keyValuePair.getValue())) {
                                    tuple.setValue(poseStampedToString(keyValuePair.getValue()));
                                    goalProperties.add(tuple);
                                } else if(isDesignator(keyValuePair.getValue())) {
                                    tuple.setValue(extractDesignatorAndGetId(
                                            designators, (MongoLogElement) keyValuePair.getValue()));
                                    designatorIds.add(tuple);
                                }
                            }
                        }
                    }
                }
            }
        }
        robotStateMessage.setDesignatorKeyToDesignatorId(designatorIds);
        robotStateMessage.setAggregatedChildDesignators(designators);
        robotStateMessage.setGoalProperties(goalProperties);
    }

    private LinkedList<Map.Entry<String, LogElement>> getDesignatorsForGoal(LogElement goal) {
        LinkedList<Map.Entry<String, LogElement>> toReturn = new LinkedList<>();
        if(!goal.getOtherObjectProperties().containsKey(Constants.PROPERTY_NAME_PARAMETERS) ||
                goal.getOtherObjectProperties().get(Constants.PROPERTY_NAME_PARAMETERS).size()!=1) {
            return toReturn;
        }
        for(Map.Entry<String, Collection<LogElement>> keyListPair : goal.getOtherObjectProperties().
                get(Constants.PROPERTY_NAME_PARAMETERS).iterator().next().getOtherObjectProperties().entrySet()) {
            for(LogElement designator :  keyListPair.getValue()) {
                toReturn.add(new AbstractMap.SimpleEntry<String, LogElement>(keyListPair.getKey(), designator));
            }
        }
        return toReturn;
    }

    private String extractDesignatorAndGetId(List<Designator> designators, MongoLogElement elementToExtract)
            throws Exception{
        Designator toReturn = mPublisher.createMessage(Designator._TYPE);
        toReturn.setId(UUID.randomUUID().toString());
        toReturn.setType(Designator.TYPE_UNDEFINED);
        LinkedList<Tuple> designatorProperties = new LinkedList<>();
        for(Map.Entry<String, Collection<String>> entries : elementToExtract.getOtherDataProperties().entrySet()) {
            if(     !entries.getKey().toLowerCase().equals(Constants.PROPERTY_NAME_DESIGNATOR_TYPE) &&
                    !entries.getKey().equals(Constants.PROPERTY_NAME_DESIGNATOR_ID) &&
                    entries.getKey().startsWith("_")) {
                continue;
            }
            for(String value : entries.getValue()) {
                if(entries.getKey().toLowerCase().equals(Constants.PROPERTY_NAME_DESIGNATOR_TYPE)) {
                    toReturn.setType(convertDesignatorTypeStringToRosEnum(value.toLowerCase()));
                } else if(entries.getKey().toLowerCase().equals(Constants.PROPERTY_NAME_DESIGNATOR_ID)) {
                    toReturn.setId(value);
                } else {
                    Tuple tuple = mPublisher.createMessage(Tuple._TYPE);
                    tuple.setName(entries.getKey());
                    tuple.setValue(value);
                    designatorProperties.add(tuple);
                }
            }
        }
        LinkedList<Tuple> subDesignatorIds = new LinkedList<>();
        for(Map.Entry<String, Collection<LogElement>> entries : elementToExtract.getOtherObjectProperties().entrySet()){
            for(LogElement potentialSubDesignator : entries.getValue()) {
                Tuple subDesignator = mPublisher.createMessage(Tuple._TYPE);
                subDesignator.setName(entries.getKey());
                if(isPose(potentialSubDesignator)) {
                    subDesignator.setValue(poseToString(potentialSubDesignator));
                    designatorProperties.add(subDesignator);
                } else if(isPoseStamped(potentialSubDesignator)) {
                    subDesignator.setValue(poseStampedToString(potentialSubDesignator));
                    designatorProperties.add(subDesignator);
                } else if(isDesignator(potentialSubDesignator)) {
                    subDesignator.setValue(
                            extractDesignatorAndGetId(designators,(MongoLogElement) potentialSubDesignator));
                    subDesignatorIds.add(subDesignator);
                } else {
                    mPublisher.logWarning("Unknown mongo sub element!");
                }
            }
        }
        toReturn.setProperties(designatorProperties);
        toReturn.setDesignatorKeyToDesignatorId(subDesignatorIds);
        designators.add(toReturn);
        return toReturn.getId();
    }

    private byte convertDesignatorTypeStringToRosEnum(String designatorType) throws Exception {
        if(designatorType.equals(Constants.PROPERTY_VALUE_ACTION_DESIGNATOR)) {
            return Designator.TYPE_ACTION;
        } else if (designatorType.equals(Constants.PROPERTY_VALUE_OBJECT_DESIGNATOR)) {
            return Designator.TYPE_OBJECT;
        } else if (designatorType.equals(Constants.PROPERTY_VALUE_LOCATION_DESIGNATOR)) {
            return Designator.TYPE_LOCATION;
        } else if (designatorType.equals(Constants.PROPERTY_VALUE_HUMAN_DESIGNATOR)) {
            return Designator.TYPE_HUMAN;
        } else if (designatorType.equals(Constants.PROPERTY_VALUE_SPEECH_DESIGNATOR)) {
            return Designator.TYPE_SPEECH;
        } else {
            throw new Exception("Unknown designator type: " + designatorType);
        }
    }

    private boolean isDesignator(LogElement element) {
        return  (element instanceof MongoLogElement) &&
                element.getOtherDataProperties().containsKey(Constants.PROPERTY_NAME_DESIGNATOR_TYPE) &&
                element.getOtherDataProperties().get(Constants.PROPERTY_NAME_DESIGNATOR_TYPE).size()==1 ||
                element.getOtherDataProperties().containsKey(Constants.PROPERTY_NAME_DESIGNATOR_TYPE.toUpperCase()) &&
                element.getOtherDataProperties().get(Constants.PROPERTY_NAME_DESIGNATOR_TYPE.toUpperCase()).size()==1;
    }

    private boolean isPose(LogElement element) {
        if(     !(element instanceof MongoLogElement) ||
                !element.getOtherObjectProperties().containsKey(Constants.PROPERTY_NAME_POSITION) ||
                element.getOtherObjectProperties().get(Constants.PROPERTY_NAME_POSITION).size()!=1 ||
                !element.getOtherObjectProperties().containsKey(Constants.PROPERTY_NAME_ORIENTATION) ||
                element.getOtherObjectProperties().get(Constants.PROPERTY_NAME_ORIENTATION).size()!=1 ||
                element.getOtherDataProperties().size()>0)
            return false;
        LogElement position = element.getOtherObjectProperties().
                get(Constants.PROPERTY_NAME_POSITION).iterator().next();
        LogElement orientation = element.getOtherObjectProperties().
                get(Constants.PROPERTY_NAME_ORIENTATION).iterator().next();
        return  position.getOtherDataProperties().containsKey(Constants.PROPERTY_NAME_POSITION_X) &&
                position.getOtherDataProperties().get(Constants.PROPERTY_NAME_POSITION_X).size()==1 &&
                position.getOtherDataProperties().containsKey(Constants.PROPERTY_NAME_POSITION_Y) &&
                position.getOtherDataProperties().get(Constants.PROPERTY_NAME_POSITION_Y).size()==1 &&
                position.getOtherDataProperties().containsKey(Constants.PROPERTY_NAME_POSITION_Z) &&
                position.getOtherDataProperties().get(Constants.PROPERTY_NAME_POSITION_Z).size()==1 &&
                orientation.getOtherDataProperties().containsKey(Constants.PROPERTY_NAME_ORIENTATION_W) &&
                orientation.getOtherDataProperties().get(Constants.PROPERTY_NAME_ORIENTATION_W).size()==1 &&
                orientation.getOtherDataProperties().containsKey(Constants.PROPERTY_NAME_ORIENTATION_X) &&
                orientation.getOtherDataProperties().get(Constants.PROPERTY_NAME_ORIENTATION_X).size()==1 &&
                orientation.getOtherDataProperties().containsKey(Constants.PROPERTY_NAME_ORIENTATION_Y) &&
                orientation.getOtherDataProperties().get(Constants.PROPERTY_NAME_ORIENTATION_Y).size()==1 &&
                orientation.getOtherDataProperties().containsKey(Constants.PROPERTY_NAME_ORIENTATION_Z) &&
                orientation.getOtherDataProperties().get(Constants.PROPERTY_NAME_ORIENTATION_Z).size()==1;
    }

    private boolean isPoseStamped(LogElement element) {
        if(     !(element instanceof MongoLogElement) ||
                !element.getOtherObjectProperties().containsKey(Constants.PROPERTY_NAME_POSE) ||
                element.getOtherObjectProperties().get(Constants.PROPERTY_NAME_POSE).size()!=1 ||
                !element.getOtherObjectProperties().containsKey(Constants.PROPERTY_NAME_HEADER) ||
                element.getOtherObjectProperties().get(Constants.PROPERTY_NAME_HEADER).size()!=1 ||
                element.getOtherDataProperties().size()>0)
            return false;
        LogElement header = element.getOtherObjectProperties().get(Constants.PROPERTY_NAME_HEADER).iterator().next();
        LogElement pose = element.getOtherObjectProperties().get(Constants.PROPERTY_NAME_POSE).iterator().next();
        return  header.getOtherDataProperties().containsKey(Constants.PROPERTY_NAME_FRAME_ID) &&
                header.getOtherDataProperties().get(Constants.PROPERTY_NAME_FRAME_ID).size()==1 &&
                isPose(pose);
    }

    private String poseToString(LogElement pose) {
        LogElement position = pose.getOtherObjectProperties().get(Constants.PROPERTY_NAME_POSITION).iterator().next();
        LogElement orientation =
                pose.getOtherObjectProperties().get(Constants.PROPERTY_NAME_ORIENTATION).iterator().next();
        return "(" +
                getFloatProperty(position, Constants.PROPERTY_NAME_POSITION_X) + "," +
                getFloatProperty(position, Constants.PROPERTY_NAME_POSITION_Y) + "," +
                getFloatProperty(position, Constants.PROPERTY_NAME_POSITION_Z) + "," +
                getFloatProperty(orientation, Constants.PROPERTY_NAME_ORIENTATION_W) + ","+
                getFloatProperty(orientation, Constants.PROPERTY_NAME_ORIENTATION_X) + ","+
                getFloatProperty(orientation, Constants.PROPERTY_NAME_ORIENTATION_Y) + ","+
                getFloatProperty(orientation, Constants.PROPERTY_NAME_ORIENTATION_Z) + ")";
    }

    private String poseStampedToString(LogElement poseStamped) {
        LogElement pose=poseStamped.getOtherObjectProperties().get(Constants.PROPERTY_NAME_POSE).iterator().next();
        LogElement header=poseStamped.getOtherObjectProperties().get(Constants.PROPERTY_NAME_HEADER).iterator().next();
        String poseString = poseToString(pose);
        return "(" + header.getOtherDataProperties().get(Constants.PROPERTY_NAME_FRAME_ID).iterator().next() + "," +
                poseString.substring(1);
    }

    private String getFloatProperty(LogElement parent, String propertyName) {
        String floatAsString = parent.getOtherDataProperties().get(propertyName).iterator().next();
        float value = Float.parseFloat(floatAsString);
        return String.format("%.3f", value);
    }

    private final MessageSender mPublisher;
}
