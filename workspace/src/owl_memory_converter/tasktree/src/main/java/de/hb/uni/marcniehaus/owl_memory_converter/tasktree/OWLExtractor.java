/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package de.hb.uni.marcniehaus.owl_memory_converter.tasktree;

import java.io.File;
import java.io.FilenameFilter;
import java.util.Collection;
import java.util.Collections;
import java.util.Map;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Set;
import java.util.Stack;
import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.io.IRIDocumentSource;
import org.semanticweb.owlapi.io.OWLOntologyDocumentSource;
import org.semanticweb.owlapi.model.*;
import org.semanticweb.owlapi.util.PriorityCollection;
import uk.ac.manchester.cs.owl.owlapi.OWLNamedIndividualImpl;
import uk.ac.manchester.cs.owl.owlapi.ParsableOWLOntologyFactory;

import javax.annotation.Nonnull;

/**
 *
 * @author marc
 */
public class OWLExtractor {
    public static OWLExtractor crateExtractor(File owlFile, 
            Set<String> filteredClasses) 
            throws Exception  {
        OWLExtractor toReturn = new OWLExtractor(owlFile);
        toReturn.createLogElements();
        toReturn.setProperties(filteredClasses);
        toReturn.addMongoDbLinks();
        return toReturn;
    }
    
    private OWLExtractor(File owlFile) throws Exception {
        String path = owlFile.getAbsolutePath();
        String[] pathParts = path.split(File.separator);
        if(pathParts.length<4)
            throw new Exception("Path must contain at least 4 parts in order to resolve the database name!");
        String databaseName = pathParts[pathParts.length-4] + "_" + pathParts[pathParts.length-3];
        mMongoExtractor = new  MongoExtractor(databaseName);
        OWLOntologyManager manager = new OWLManager().get();
        PriorityCollection<OWLOntologyFactory> factories = manager.getOntologyFactories();
        OWLOntologyFactory originalFactory = null;
        for(OWLOntologyFactory factory : factories) {
            if(factory instanceof ParsableOWLOntologyFactory) {
                originalFactory = factory;
            }
        }
        if(originalFactory==null) {
            throw new Exception("No ParsableOWLOntologyFactory found!");
        }
        factories.add(new RosOntologyFactory(originalFactory));
        mOntology=manager.loadOntologyFromOntologyDocument(owlFile);
    }
    
    public Collection<Task> getRootTasks() throws Exception {
        LinkedList<Task> toReturn = new LinkedList<>();
        for(OWLLogElement t : mElements.values()) {
            if(t instanceof Task && 
                    ((Task) t).getParentTask()==null) {
                toReturn.add((Task) t);
            }
        }
        return toReturn;        
    }
    
    public Map<String, Integer> getClassCount(boolean useContext) {
        Map<String, Integer> toReturn = new HashMap<>();
        for(OWLLogElement l : mElements.values()) {
            String className = useContext && l instanceof Task ? 
                    ((Task) l).getContext() : l.getOwlClassName();
            Integer count = toReturn.get(className);
            count = count==null ? 0 : count;
            toReturn.put(className, ++count);
        }
        return toReturn;
    }
    
    private Map<IRI,  OWLLogElement> createLogElements() throws Exception {
        mElements = new HashMap<>();
        for(OWLIndividual ind : mOntology.getIndividualsInSignature()) {
            if(ind.isNamed() && isTask(ind)) {
                mElements.put(ind.asOWLNamedIndividual().getIRI(), new Task());
            } else if(ind.isNamed() && isDesignator(ind)) {
                mElements.put(ind.asOWLNamedIndividual().getIRI(), 
                        new OWLDesignator());
            } else {
                mElements.put(ind.asOWLNamedIndividual().getIRI(), new
                    OWLLogElement());
            }
        } 
        return mElements;
    }
            
    private void setProperties(Set<String> filteredElements) throws Exception {
        for(IRI elementIRI : new HashSet<>(mElements.keySet())) {
            OWLLogElement element = mElements.get(elementIRI);
            setPropertiesForLogElement(elementIRI, element);
            if(element instanceof Task) {
                setPropertiesForTask(elementIRI, 
                        (Task) element, filteredElements);
            }
        }
    }
    
    private void setPropertiesForTask(IRI taskIRI, Task task, 
            Set<String> filteredElements) throws Exception {
        OWLNamedIndividual individual = 
                getIndividualFromIRI(taskIRI, true);
        if(filteredElements.contains(getClassName(individual))) {
            return;
        }
        task.setContext(getTaskContext(individual));
        LinkedList<Task> subTasks = getFilteredSubtasks(individual, 
                task, filteredElements);
        for(Task subtask : subTasks) {  
            subtask.setParentTask(task);
        }
        task.setSubtasks(subTasks);
        task.getOtherObjectProperties().put(Constants.PROPERTY_SUBTASK, 
                new LinkedList<LogElement>(subTasks));
    }
    
    private void addreversedSubtasks(Stack<OWLNamedIndividual> stack, 
            OWLNamedIndividual task) throws Exception {        
        LinkedList<OWLNamedIndividual> reversedSubtasks = getSubtasks(task);
        Collections.reverse(reversedSubtasks);
        for(OWLNamedIndividual subtask : reversedSubtasks) {
            stack.push(subtask);
        }
    }
    
    private LinkedList<Task> getFilteredSubtasks(OWLNamedIndividual task, 
            Task taskObject, Set<String> filteredElements) throws Exception {
        LinkedList<Task> toReturn = new LinkedList<>();
        Stack<OWLNamedIndividual> subtasks = new Stack<>();
        addreversedSubtasks(subtasks, task);
        while(!subtasks.empty()) {         
            OWLNamedIndividual subTask = subtasks.pop();
            OWLLogElement nextSubtaskObject=mElements.get(subTask.getIRI());
            if(nextSubtaskObject==null || 
                    !(nextSubtaskObject instanceof Task)){
                throw new Exception("Invalid reference");
            }
            ((Task) nextSubtaskObject).setParentTask(taskObject);
            if(filteredElements.contains(getClassName(subTask))) {
                addreversedSubtasks(subtasks, subTask);                
            } else {
                toReturn.add((Task) nextSubtaskObject);
            }
        }
        return toReturn;
    }
     
    private void setPropertiesForLogElement(IRI taskIRI, OWLLogElement element) 
            throws Exception {
        element.setOwlInstanceName(taskIRI.getShortForm());
        OWLIndividual owlElement = getIndividualFromIRI(taskIRI, true);
        element.setOwlClassName(getClassName(owlElement));
        setOtherDataProperties(element, owlElement);
        setOtherObjectProperties(element, owlElement);
        LinkedList<String> classNameList = new LinkedList<>();
        classNameList.add(element.getOwlClassName());
        LinkedList<String> instanceNameList = new LinkedList<>();
        instanceNameList.add(element.getOwlInstanceName());
        element.getOtherDataProperties().put("OWLClassName", classNameList);
        element.getOtherDataProperties().put("OWLName", instanceNameList);
    }
    
    private void setOtherDataProperties(OWLLogElement element, 
            OWLIndividual owlElement) {
        Map<String, Collection<String> > properties = new HashMap<>();
        for(String property : getDataPropertyShortNames(owlElement)) {
            if(!Constants.OTHER_PROPERTY_EXCLUDES.contains(property)) {
                properties.put(property, 
                        getDataPropertyValues(owlElement, property));
            }
        }
        element.setOtherDataProperties(properties);
    }
    
    private void setOtherObjectProperties( 
            OWLLogElement element, OWLIndividual owlElement) throws Exception {
        Map<String, Collection<LogElement> > properties = new HashMap<>();
        for(String property :getObjectPropertyShortNames(owlElement)){
            if(!Constants.OTHER_PROPERTY_EXCLUDES.contains(property)) {
                LinkedList<OWLNamedIndividual> owlIndividuals = 
                        getObjectPropertyValues(owlElement, property);
                LinkedList<LogElement> individuals = new LinkedList<>();
                for(OWLNamedIndividual ind : owlIndividuals) {
                    OWLLogElement correspondingLogElement = 
                            mElements.get(ind.getIRI());
                    if(correspondingLogElement==null) {
                        correspondingLogElement = new OWLLogElement();
                        correspondingLogElement.setOwlInstanceName(
                                ind.getIRI().getShortForm());
                        correspondingLogElement.setOwlClassName(
                                "Unknown Class!");
                        mElements.put(ind.getIRI(), correspondingLogElement);
                    }
                    individuals.add(correspondingLogElement);
                }
                properties.put(property, individuals);
            }
        }
        element.setOtherObjectProperties(properties); 
    }
          
    private String getClassName(OWLIndividual owlElement) {
        for(OWLClassAssertionAxiom axiom : 
                mOntology.getClassAssertionAxioms(owlElement)) {
            OWLClassExpression classExpression = axiom.getClassExpression();
            if(!classExpression.toString().equals(
                    Constants.IRI_NAMED_INDIVIDUAL)) {
                return classExpression.asOWLClass().getIRI().getShortForm();
            }
        }
        return "";
    }            
            
    private OWLNamedIndividual getIndividualFromIRI(IRI iri, boolean throwError) 
            throws Exception {
        Set<OWLEntity> entities = mOntology.getEntitiesInSignature(iri);
        for(OWLEntity entity : entities) {
            if(entity.isOWLNamedIndividual()) {
                return (OWLNamedIndividual) entity;
            }
        }
        if(throwError) {
            throw new Exception("Invalid IRI: " + iri + ": found no named individual");
        } else {
            return null;
        }
    }
    
    private String getTaskContext(OWLIndividual task) {
        LinkedList<String> propertyValues = getDataPropertyValues( 
                task, Constants.PROPERTY_NAME_TASK_CONTEXT);
        return propertyValues.size()>0 ? propertyValues.getFirst() : null;
    }
    
    private boolean isTask(OWLIndividual individual) {
        return getTaskContext(individual)!=null;
    }            
    
    private boolean isDesignator(OWLIndividual individual) {
        return getClassName(individual).equals(Constants.CLASS_NAME_DESIGNATOR);
    }
            
    private LinkedList<String> getObjectPropertyShortNames(
            OWLIndividual subject) {
        LinkedList<String> toReturn = new LinkedList<>();
        
        for(OWLObjectPropertyAssertionAxiom axiom : 
                mOntology.getObjectPropertyAssertionAxioms(subject)) {
            toReturn.add(axiom.getProperty().asOWLObjectProperty().getIRI().
                    getShortForm());            
        }
        //workaround for wrong owl files:
        
        for(OWLAnnotationAssertionAxiom axiom : mOntology.
                getAnnotationAssertionAxioms(subject.
                    asOWLNamedIndividual().getIRI())) {
            if(axiom.getAnnotation().getValue() instanceof IRI) {
                toReturn.add(axiom.getAnnotation().getProperty().getIRI().
                        getShortForm());
            }
        }
        
        return toReturn;
    }

    private LinkedList<String> getDataPropertyShortNames(OWLIndividual subject){
        LinkedList<String> toReturn = new LinkedList<>();
        
        for(OWLDataPropertyAssertionAxiom axiom : 
                mOntology.getDataPropertyAssertionAxioms(subject)) {
            toReturn.add(axiom.getProperty().asOWLDataProperty().getIRI().
                    getShortForm());            
        }
        
        //workaround for wrong owl files:
        
        for(OWLAnnotationAssertionAxiom axiom : mOntology.
                getAnnotationAssertionAxioms(subject.
                    asOWLNamedIndividual().getIRI())) {
            if(axiom.getAnnotation().getValue() instanceof OWLLiteral) {
                toReturn.add(axiom.getAnnotation().getProperty().getIRI().
                        getShortForm());
            }
        }
        
        return toReturn;
    }
    
    private LinkedList<OWLNamedIndividual> getObjectPropertyValues(
            OWLIndividual subject, 
            String shortPropertyName) throws Exception {
        LinkedList<OWLNamedIndividual> toReturn = new LinkedList<>();
        for(OWLObjectPropertyAssertionAxiom axiom : 
                mOntology.getObjectPropertyAssertionAxioms(subject)) {
            if(axiom.getProperty().asOWLObjectProperty().getIRI().
                    getShortForm().equals(shortPropertyName)) {
                if(!axiom.getObject().isNamed()) {
                    throw new Exception("All object properties must be named!");
                }                
                toReturn.add(axiom.getObject().asOWLNamedIndividual());
            }
        }
                
        //workaround for wrong owl files:
        
        for(OWLAnnotationAssertionAxiom axiom : mOntology.
                getAnnotationAssertionAxioms(subject.
                    asOWLNamedIndividual().getIRI())) {
            if(axiom.getAnnotation().getValue() instanceof IRI && 
                    axiom.getAnnotation().getProperty().getIRI().getShortForm().
                            equals(shortPropertyName)) {
                IRI iri = axiom.getAnnotation().getValue().asIRI().get();
                OWLNamedIndividual individual = getIndividualFromIRI(iri,false);
                toReturn.add(individual!=null ? individual : 
                        new OWLNamedIndividualImpl(iri));
            }
        }
        
        return toReturn;
    }
    
    private LinkedList<String> getDataPropertyValues(OWLIndividual subject, 
            String shortPropertyName)  {
        
        LinkedList<String> toReturn = new LinkedList<>();        
        for(OWLDataPropertyAssertionAxiom axiom : 
                mOntology.getDataPropertyAssertionAxioms(subject)) {
            OWLDataProperty property = axiom.getProperty().asOWLDataProperty();
            if(property.getIRI().getShortForm().equals(shortPropertyName)) {
                toReturn.add(axiom.getObject().getLiteral());
            } 
        }
        
                //workaround for wrong owl files:
        
        for(OWLAnnotationAssertionAxiom axiom : mOntology.
                getAnnotationAssertionAxioms(subject.
                    asOWLNamedIndividual().getIRI())) {
            if(axiom.getAnnotation().getValue() instanceof OWLLiteral && 
                    axiom.getAnnotation().getProperty().getIRI().getShortForm().
                            equals(shortPropertyName)) {
                toReturn.add(axiom.getAnnotation().getValue().
                        asLiteral().get().getLiteral());
            }
        }
        
        return toReturn;
    }
    
    private void addMongoDbLinks() 
            throws Exception {
        for(OWLLogElement element : mElements.values()) {
            if(element instanceof OWLDesignator) {
                MongoLogElement mongoDesignator = mMongoExtractor.getDesignator(
                              element.getOwlInstanceName());
                if(mongoDesignator!=null) {
                    ((OWLDesignator) element).setMongoRepresentation(
                            mongoDesignator);
                }
            }
        }
    }
    
    private LinkedList<OWLNamedIndividual> getSubtasks(
            OWLNamedIndividual individual) throws Exception {
                OWLNamedIndividual nextSubTask = null;
        for(OWLNamedIndividual subTask : getObjectPropertyValues(
                individual, Constants.PROPERTY_SUBTASK)) {
            if(!mElements.containsKey(subTask.getIRI())) {
                throw new Exception("Referenced subtask is not described!");
            }            
            if(getObjectPropertyValues(subTask, 
                    Constants.PROPERTY_PREVIOUS_TASK).isEmpty()) {
                nextSubTask = subTask;
            }
        }
        LinkedList<OWLNamedIndividual> subTasks = new LinkedList<>();
        while(nextSubTask!=null) {            
            subTasks.add(nextSubTask);
            LinkedList<OWLNamedIndividual> nextSubTasks = 
                    getObjectPropertyValues(nextSubTask, 
                    Constants.PROPERTY_NEXT_TASK);
            if(nextSubTasks.size()>1) {
                throw new Exception("Invalid number of next subtasks!");                
            }
            nextSubTask = nextSubTasks.isEmpty() ? null : 
                    nextSubTasks.getFirst();
        }
        return subTasks;
    }

    private class RosOntologyFactory implements OWLOntologyFactory {

        public RosOntologyFactory(OWLOntologyFactory proxyFactory) {
            mProxyFactory = proxyFactory;
        }

        @Nonnull
        @Override
        public OWLOntology createOWLOntology(OWLOntologyManager owlOntologyManager,
                                             OWLOntologyID owlOntologyID,
                                             IRI iri,
                                             OWLOntologyCreationHandler owlOntologyCreationHandler)
                                            throws OWLOntologyCreationException {
            return mProxyFactory.createOWLOntology(owlOntologyManager, owlOntologyID, iri, owlOntologyCreationHandler);
        }

        @Nonnull
        @Override
        public OWLOntology loadOWLOntology(@Nonnull OWLOntologyManager owlOntologyManager,
                                           @Nonnull OWLOntologyDocumentSource owlOntologyDocumentSource,
                                           @Nonnull OWLOntologyCreationHandler owlOntologyCreationHandler,
                                           @Nonnull OWLOntologyLoaderConfiguration owlOntologyLoaderConfiguration)
                                            throws OWLOntologyCreationException {

            if(owlOntologyDocumentSource.getDocumentIRI().toString().toLowerCase().startsWith("package://")) {
                owlOntologyDocumentSource = new IRIDocumentSource(IRI.create(
                        "file://" + getRosPackageFilePath(owlOntologyDocumentSource.getDocumentIRI().toString())));
            }
            return mProxyFactory.loadOWLOntology(owlOntologyManager, owlOntologyDocumentSource,
                    owlOntologyCreationHandler, owlOntologyLoaderConfiguration);
        }

        @Override
        public boolean canCreateFromDocumentIRI(@Nonnull IRI iri) {
            return mProxyFactory.canCreateFromDocumentIRI(iri);
        }

        @Override
        public boolean canLoad(@Nonnull OWLOntologyDocumentSource owlOntologyDocumentSource) {
            if (owlOntologyDocumentSource.getDocumentIRI().toString().toLowerCase().startsWith("package://")) {
                return getRosPackageFilePath(owlOntologyDocumentSource.getDocumentIRI().toString())!=null;
            }
            return mProxyFactory.canLoad(owlOntologyDocumentSource);
        }

        private String getRosPackageFilePath(String packageUrl) {
            final String fileSuffix = packageUrl.substring("package://".length());
            Map<String, String> envVars = System.getenv();
            final String packagePathEnvVar = "ROS_PACKAGE_PATH";
            if(!envVars.containsKey(packagePathEnvVar))
                return null;
            for(String pathPrefix : envVars.get(packagePathEnvVar).split(File.pathSeparator)) {
                File pathFile = new File(pathPrefix);
                if(!pathFile.exists())
                    continue;
                String foundFile = findFileInDirectory(pathFile, fileSuffix);
                if(foundFile!=null)
                    return foundFile;
            }
            return null;
        }

        private String findFileInDirectory(File directory, final String fileSuffix) {
            for(File file : directory.listFiles()) {
                if(file.isDirectory()) {
                    String foundFile = findFileInDirectory(file, fileSuffix);
                    if(foundFile!=null)
                        return foundFile;
                } else if(file.getAbsolutePath().endsWith(fileSuffix)) {
                    return file.getAbsolutePath();
                }
            }
            return null;
        }

        private OWLOntologyFactory mProxyFactory;
    }
    
    private final OWLOntology mOntology;
    private Map<IRI, OWLLogElement> mElements;
    private final MongoExtractor mMongoExtractor;
}
