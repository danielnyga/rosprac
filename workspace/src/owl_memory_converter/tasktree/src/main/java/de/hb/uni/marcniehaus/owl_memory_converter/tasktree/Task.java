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

import java.util.ArrayList;
import java.util.Collection;

public class Task extends OWLLogElement {
    public ArrayList<Task> getSubTasks() {
        return mSubtasks;
    }
    
    void setSubtasks(Collection<Task> subtasks) {
        mSubtasks = new ArrayList<>(subtasks);
    }
    
    public Task getParentTask() {
        return mParentTask;
    }
    
    void setParentTask(Task parentTask) {
        mParentTask = parentTask;
    }
    
    public String getContext() {
        return mContext;
    }
    
    public void setContext(String context) {
        mContext = context;
    }
    
    @Override
    public String toString() {
        return mContext;
    }
    
    private Task mParentTask = null;
    private ArrayList<Task> mSubtasks = new ArrayList<>();
    private String mContext = "";
}
