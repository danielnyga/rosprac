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

/**
 *
 * @author marc
 */
public class OWLLogElement extends LogElement {
    @Override
    public String toString() {
        return mOwlClassName;
    }
    
    public String getOwlClassName() {
        return mOwlClassName;
    }
    
    void setOwlClassName(String owlClassName) {
        mOwlClassName = owlClassName;
    }
    
    public String getOwlInstanceName() {
        return mOwlInstanceName;
    }
    
    void setOwlInstanceName(String owlInstanceName) {
        mOwlInstanceName = owlInstanceName;
    }
    
    private String mOwlInstanceName = "";
    private String mOwlClassName = "";

}
