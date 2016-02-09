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

import static com.mongodb.client.model.Filters.*;

import com.mongodb.MongoClient;
import com.mongodb.client.MongoCollection;
import com.mongodb.client.MongoDatabase;

import java.io.StringReader;
import java.util.*;

import com.sun.jna.Library;
import com.sun.jna.Native;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.PointerByReference;
import org.bson.Document;

import javax.json.Json;
import javax.json.stream.JsonParser;

/**
 *
 * @author marc
 */
public class MongoExtractor {
    public MongoExtractor(String databaseName) throws Exception {
        try {
            mMongodb = new CDriver();
          //  throw new UnsupportedOperationException();
        } catch(Exception e) {
            e.printStackTrace();
            System.err.println("Initializing the C mongodb driver failed! Using the java driver...\n"+
                    "(Goal properties will not be available!)");
            mMongodb = new JavaDriver();
        }
        mMongodb.connect(databaseName, "logged_designators");
    }
    
    MongoLogElement getDesignator(String designatorId) throws Exception {
        if(mCachedDesignators.containsKey(designatorId)) {
            return mCachedDesignators.get(designatorId);
        }
        LinkedList<String> designators = mMongodb.find("designator._id", designatorId);
        MongoLogElement toReturn = new MongoErrorElement();
        if(designators.size()!=1)
            return toReturn;

        JsonParser parser = Json.createParser(new StringReader(designators.getFirst()));
        Stack<String> keyStack = new Stack<>();
        Stack<MongoLogElement> objectStack = new Stack<>();
        while (parser.hasNext()) {
            JsonParser.Event event = parser.next();
            switch(event) {
                case START_ARRAY:
                    throw new UnsupportedOperationException();
                case END_ARRAY:
                    throw new UnsupportedOperationException();
                case START_OBJECT:
                    String key = keyStack.empty() ? null : keyStack.pop();
                    MongoLogElement element = new MongoLogElement();
                    if(key!=null && !objectStack.empty() && !key.startsWith("_id") && !key.startsWith("__recorded")) {
                        addToMultiMap(objectStack.peek().getOtherObjectProperties(), key, element);
                    }
                    objectStack.push(element);
                    break;
                case END_OBJECT:
                    toReturn = objectStack.pop();
                    break;
                case KEY_NAME:
                    keyStack.push(parser.getString());
                    break;
                default:
                    addToMultiMap(objectStack.peek().getOtherDataProperties(), keyStack.pop(), parser.getString());
            }
        }
        mCachedDesignators.put(designatorId, toReturn);
        return reorderParametersIfNecessary(toReturn);
    }

    private <T> void addToMultiMap(Map<String, Collection<T> > multiMap, 
            String key, T value) {
        Collection<T> keyProperties;
        if(multiMap.containsKey(key)) {
            keyProperties = multiMap.get(key);
        } else {
            keyProperties = new LinkedList<>();
            multiMap.put(key, keyProperties);
        }
        keyProperties.add(value);
    }

    private MongoLogElement reorderParametersIfNecessary(MongoLogElement mongoElement) {
        if(     !mongoElement.getOtherObjectProperties().containsKey(Constants.PROPERTY_NAME_DESIGNATOR) ||
                mongoElement.getOtherObjectProperties().get(Constants.PROPERTY_NAME_DESIGNATOR).size()!=1) {
            return mongoElement;
        }
        LogElement designator = mongoElement.getOtherObjectProperties().
                get(Constants.PROPERTY_NAME_DESIGNATOR).iterator().next();
        if(     !designator.getOtherObjectProperties().containsKey(Constants.PROPERTY_NAME_PARAMETERS) ||
                designator.getOtherObjectProperties().get(Constants.PROPERTY_NAME_PARAMETERS).size()!=1) {
            return mongoElement;
        }
        LogElement properties = designator.getOtherObjectProperties().
                get(Constants.PROPERTY_NAME_PARAMETERS).iterator().next();
        reorderDesignatorRecursively(properties, null);
        return mongoElement;
    }

    private void reorderDesignatorRecursively(LogElement element, LogElement parent) {
        if(element.getOtherObjectProperties().containsKey("")) {
            for(LogElement child : new LinkedList<>(element.getOtherObjectProperties().get(""))) {
                reorderDesignatorRecursively(child, element);
            }
        }
        boolean isLeaf = element.getOtherObjectProperties().size()==0 &&
                         element.getOtherDataProperties().entrySet().size()==1 &&
                         element.getOtherDataProperties().containsKey("") &&
                         element.getOtherDataProperties().get("").size()==2;
        boolean isRole = element.getOtherObjectProperties().entrySet().size()==1 &&
                         element.getOtherObjectProperties().containsKey("") &&
                         element.getOtherObjectProperties().get("").size()==1 &&
                         element.getOtherDataProperties().entrySet().size()==1 &&
                         element.getOtherDataProperties().containsKey("") &&
                         element.getOtherDataProperties().get("").size()==1;
        if(isLeaf && parent!=null) {
            Iterator<String> iterator = element.getOtherDataProperties().get("").iterator();
            String key = iterator.next();
            String value = iterator.next();
            addToMultiMap(parent.getOtherDataProperties(), key, value);
            parent.getOtherObjectProperties().get("").remove(element);
        } else if(isRole && parent!=null) {
            String key = element.getOtherDataProperties().get("").iterator().next();
            LogElement value = element.getOtherObjectProperties().get("").iterator().next();
            addToMultiMap(parent.getOtherObjectProperties(), key, value);
            parent.getOtherObjectProperties().get("").remove(element);
        }
    }

    private interface MongodbDriver {
        void connect(String database, String collection) throws Exception;
        LinkedList<String> find(String key, String value) throws Exception;
    }

    /*
    The C driver is necessary because the java driver cannot cope with more than one empty key.
    However, the parameters of a goal are stored as empty keys...
     */
    private class CDriver implements MongodbDriver {
        public CDriver()  throws Exception{
            System.setProperty("jna.nosys", "true");
            if(mNativeStdlib==null) {
                mNativeStdlib = (Stdlib) Native.loadLibrary("c", Stdlib.class);
            }
            // Set the locale; otherwise, json formattig will fail with a german locale (uses decimal comma)
            final int LC_NUMERIC = 1;
            checkForError(mNativeStdlib.setlocale(LC_NUMERIC, "C"));
            if(mNativeLibrary==null) {
                mNativeLibrary = (MongoC) Native.loadLibrary("mongoc-1.0", MongoC.class);
            }
        }

        @Override
        public void connect(String databaseName, String collectionName)  throws Exception{
            try {
                mNativeLibrary.mongoc_init();
                mClient = mNativeLibrary.mongoc_client_new("mongodb://localhost:27017/");
                checkForError(mClient);
                mCollection = mNativeLibrary.mongoc_client_get_collection(mClient, databaseName, collectionName);
                checkForError(mCollection);
            } catch (Throwable t) {
                if(mCollection!=null) {
                    mNativeLibrary.mongoc_collection_destroy(mCollection);
                    mCollection = null;
                }
                if(mClient!=null) {
                    mNativeLibrary.mongoc_client_destroy(mClient);
                    mClient = null;
                }
                mNativeLibrary.mongoc_cleanup();
            }
        }

        @Override
        public LinkedList<String> find(String key, String value)  throws Exception{
            LinkedList<String> toReturn = new LinkedList<>();
            Pointer query = null;
            Pointer cursor = null;
            Pointer str = null;
            try {
                query = mNativeLibrary.bson_new();
                checkForError(query);
                checkForError(mNativeLibrary.bson_append_utf8(query, key, key.length(), value, value.length()));
                cursor = mNativeLibrary.mongoc_collection_find(mCollection, 0, 0, 0, 0, query, null, null);
                checkForError(cursor);
                PointerByReference docReference = new PointerByReference();
                while (mNativeLibrary.mongoc_cursor_next(cursor, docReference)) {
                    str = mNativeLibrary.bson_as_json(docReference.getValue(), null);
                    checkForError(str);
                    String s = str.getString(0);
                    toReturn.add(new String(s));
                    mNativeLibrary.bson_free(str);
                    str=null;
                }
            } finally {
                if(query!=null) {
                    mNativeLibrary.bson_destroy(query);
                }
                if(cursor!=null) {
                    mNativeLibrary.mongoc_cursor_destroy(cursor);
                }
                if(str!=null) {
                    mNativeLibrary.bson_free(str);
                }
            }
            return toReturn;
        }

        @Override
        protected void finalize() throws Throwable {
            if(mCollection!=null) {
                mNativeLibrary.mongoc_collection_destroy(mCollection);
            }
            if(mClient!=null) {
                mNativeLibrary.mongoc_client_destroy(mClient);
            }
            mNativeLibrary.mongoc_cleanup();
        }

        public void checkForError(Pointer result) throws Exception {
            if(result==null) {
                throw new Exception("Native call failed!");
            }
        }

        public void checkForError(boolean result) throws Exception {
            if(!result) {
                throw new Exception("Native call failed!");
            }
        }

        private Pointer mClient;
        private Pointer mCollection;
    }

    private class JavaDriver implements MongodbDriver {

        @Override
        public void connect(String database, String collection) {
            MongoClient mongoClient = new MongoClient("localhost", 27017);
            MongoDatabase db = mongoClient.getDatabase(database);
            mDesignatorCollection = db.getCollection(collection);
        }

        @Override
        public LinkedList<String> find(String key, String value) {
            LinkedList<String> toReturn = new LinkedList<>();
            for (Document doc : mDesignatorCollection .find(eq(key, value))) {
                toReturn.add(doc.toJson());
            }
            return toReturn;
        }

        private MongoCollection<Document> mDesignatorCollection = null;
    }

    private interface MongoC extends Library {
        void mongoc_init();
        Pointer mongoc_client_new(String uri);
        Pointer mongoc_client_get_collection(Pointer client, String database, String collection);
        Pointer bson_new();
        boolean bson_append_utf8 (Pointer b, String key, int key_length, String val, int val_length);
        Pointer mongoc_collection_find(Pointer collection, int queryMode, int skip, int limits,
                                       int batch_size, Pointer query, Pointer fields, Pointer prefs);
        boolean mongoc_cursor_next(Pointer cursor, PointerByReference doc);
        Pointer bson_as_json(Pointer bson, Pointer arg);
        void bson_free(Pointer str);
        void bson_destroy(Pointer query);
        void mongoc_cursor_destroy(Pointer cursor);
        void mongoc_collection_destroy(Pointer collection);
        void mongoc_client_destroy(Pointer client);
        void mongoc_cleanup();
    }

    private interface Stdlib extends Library {
        Pointer setlocale(int locale, String value);
    }

    private final Map<String, MongoLogElement> mCachedDesignators = new HashMap<>();
    private MongodbDriver mMongodb;
    private static MongoC mNativeLibrary = null;
    private static Stdlib mNativeStdlib = null;
}
