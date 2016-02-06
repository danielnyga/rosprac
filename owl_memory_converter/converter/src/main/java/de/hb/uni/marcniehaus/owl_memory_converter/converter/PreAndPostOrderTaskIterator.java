/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package de.hb.uni.marcniehaus.owl_memory_converter.converter;

import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.Task;
import java.util.ArrayList;
import java.util.Iterator;

/**
 *
 * @author marc
 */
public class PreAndPostOrderTaskIterator implements Iterator<Task>{

    public PreAndPostOrderTaskIterator(Task root) {
        mNextTask = root;
        mNextTaskFinished = false;
    }
        
    @Override
    public boolean hasNext() {
        return mNextTask!=null;
    }

    @Override
    public Task next() {
        mCurrentTask = mNextTask;
        mCurrentTaskFinished = mNextTaskFinished;
        if(mNextTaskFinished) {
            if(mNextTask.getParentTask()==null) {
                mNextTask = null;
                return mNextTask;
            }
            ArrayList<Task> siblingTasks =
                mNextTask.getParentTask().getSubTasks();
            int index = siblingTasks.indexOf(mNextTask);
            if(index==siblingTasks.size()-1) {
                mNextTask = mNextTask.getParentTask();
                mNextTaskFinished = true;
            } else {
                mNextTask = siblingTasks.get(index+1);
                mNextTaskFinished = false;
            }
        } else {
            if(mNextTask.getSubTasks().isEmpty()) {
                mNextTaskFinished = true;
            } else {
                mNextTask = mNextTask.getSubTasks().get(0);
                mNextTaskFinished = false;
            }
        }
        return mCurrentTask;
    }

    @Override
    public void remove() {
        throw new UnsupportedOperationException("Not supported.");
    } 
    
    public boolean currentHasBeenVisitedBefore() {
        return mCurrentTaskFinished;
    }
    
    public Task current() {
        return mCurrentTask;
    }
    
    private Task mCurrentTask;
    private boolean mCurrentTaskFinished;
    private Task mNextTask;
    private boolean mNextTaskFinished;
}
