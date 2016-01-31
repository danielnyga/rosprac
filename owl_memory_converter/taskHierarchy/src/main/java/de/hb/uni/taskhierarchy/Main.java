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

package de.hb.uni.taskhierarchy;
import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.OWLExtractor;
import java.io.File;
import java.util.HashSet;
import javax.swing.JFileChooser;
import javax.swing.JOptionPane;
import javax.swing.filechooser.FileFilter;

public class Main {
    public static void main( String[] args ) throws Exception {      
        File owlFile;
    	if(args.length==1) {
            owlFile = new File(args[0]);
        } else {
            JFileChooser fileChooser = new JFileChooser();
            fileChooser.setFileSelectionMode(JFileChooser.FILES_ONLY);
            fileChooser.setFileFilter(new FileFilter() {
                @Override
                public boolean accept(File file) {
                    return file.getName().endsWith("owl") || file.isDirectory();                    
                }
                @Override
                public String getDescription() {
                    return "OWL Files";
                }
            });        
            if(fileChooser.showOpenDialog(fileChooser)!=
                    JFileChooser.APPROVE_OPTION) {
                return;
            }
            owlFile = fileChooser.getSelectedFile();
        }
        int result = JOptionPane.showConfirmDialog(null, 
                "Show Failure Handling/With Deisgnators", "Question", 
                JOptionPane.YES_NO_CANCEL_OPTION);
        HashSet<String> excludedClasses = new HashSet<>();    
        switch(result) {
            default:
                throw new Exception("Unsupported Operation");            
            case JOptionPane.CANCEL_OPTION:
                return;
            case JOptionPane.NO_OPTION: 
                excludedClasses.add("WithFailureHandling");
                excludedClasses.add("WithDesignators");
                break;                
            case JOptionPane.YES_OPTION:
        }                
        MainFrame.show(OWLExtractor.crateExtractor(owlFile, excludedClasses));
    }
}
