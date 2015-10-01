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
import com.mongodb.client.FindIterable;
import com.mongodb.client.MongoCollection;
import com.mongodb.client.MongoDatabase;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import org.bson.Document;

/**
 *
 * @author marc
 */
public class MongoExtractor {
    public MongoExtractor() throws Exception {
        mMongoCLient = new MongoClient("localhost", 27017);
        MongoDatabase db = mMongoCLient.getDatabase("experiments");
        mDesignatorCollection = db.getCollection("pnp-logged_designators");
        
    }
    
    MongoLogElement getDesignator(String designatorId) throws Exception {
        if(mCachedDesignators.containsKey(designatorId)) {
            return mCachedDesignators.get(designatorId);
        }
        FindIterable<Document> results = mDesignatorCollection.
                find(eq("designator._id", designatorId));
        Document designator = results.first();
        if(designator==null) {
            return new MongoErrorElement();
        }
        MongoLogElement toReturn = parseDocument(designator);
        mCachedDesignators.put(designatorId, toReturn);
        return toReturn;
    }
    
    MongoLogElement parseDocument(Document document) {
        Map<String, Collection<String> > dataProperties = new HashMap<>();
        Map<String, Collection<LogElement> > objectProperties = new HashMap<>();
        
        for(Map.Entry<String, Object> entry : document.entrySet()) {
            if(entry.getValue() instanceof Document) {
                addToMultiMap(objectProperties, entry.getKey(), 
                        parseDocument((Document) entry.getValue()));
            } else {
                addToMultiMap(dataProperties, entry.getKey(), 
                        entry.getValue().toString());
            }
        }
        MongoLogElement toReturn = new MongoLogElement();
        toReturn.setOtherDataProperties(dataProperties);
        toReturn.setOtherObjectProperties(objectProperties);
        return toReturn;
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
    
    
    private final MongoClient mMongoCLient;
    private final MongoCollection<Document> mDesignatorCollection;
    private final Map<String, MongoLogElement> mCachedDesignators = new HashMap<>();    
}
