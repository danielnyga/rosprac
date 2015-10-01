package de.hb.uni.marcniehaus.owl_memory_converter.converter;

import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.OWLExtractor;
import java.io.File;
import java.util.HashSet;
import java.util.LinkedList;
import robot_memory.RobotState;

public class OwlConverter {    
    
    public OwlConverter(MessageSender sender) {
        mPublisher = sender;
    }
    
    public void convert(String owlFileName) throws Exception {
        System.out.println("Extract from file " + owlFileName);
        OWLExtractor ex = OWLExtractor.crateExtractor(
                new File(owlFileName), new HashSet<String>());
        RobotState s = mPublisher.createMessage();
        s.setErrors(new LinkedList<String>());
        s.setTaskId("foo bar");
        s.setSequenceNumber(sequenceNumber);
        mPublisher.send(s);
        mPublisher.send(s);
    }

    public interface MessageSender {
        public RobotState createMessage();
        public void send(RobotState state);
    }
    
    private final MessageSender mPublisher;
    private int sequenceNumber = 0;
}