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

import java.util.HashSet;

/**
 *
 * @author marc
 */
public class Constants {
    public static final String CLASS_NAME_DESIGNATOR = "CRAMDesignator";
    public static final String IRI_NAMED_INDIVIDUAL = "owl:namedIndividual";
    public static final String PROPERTY_NAME_TASK_CONTEXT = "taskContext";    
    public static final String PROPERTY_SUBTASK = "subAction";
    public static final String PROPERTY_NEXT_TASK = "nextAction";
    public static final String PROPERTY_PREVIOUS_TASK = "previousAction";
    public static final HashSet<String> OTHER_PROPERTY_EXCLUDES = 
            createOtherPropertyExcludesSet();
    
    private static HashSet<String> createOtherPropertyExcludesSet() {
        HashSet<String> toReturn = new HashSet<>();
        toReturn.add(PROPERTY_SUBTASK);
        toReturn.add(PROPERTY_NEXT_TASK);
        toReturn.add(PROPERTY_PREVIOUS_TASK);
        return toReturn;
    }
}
