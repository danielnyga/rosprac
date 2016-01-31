/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package de.hb.uni.taskhierarchy;

import de.hb.uni.marcniehaus.owl_memory_converter.tasktree.OWLExtractor;
import java.awt.HeadlessException;
import java.awt.Window;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.Map;
import java.util.Vector;
import javax.swing.JDialog;
import javax.swing.JFileChooser;
import javax.swing.JOptionPane;
import javax.swing.filechooser.FileFilter;
import javax.swing.table.DefaultTableModel;

/**
 *
 * @author marc
 */
public class StatisticsPanel extends javax.swing.JPanel {

    /**
     * Creates new form StatisticsPanel
     */
    public StatisticsPanel() {
        initComponents();
    }
    
    public void setValues(OWLExtractor extractor, 
            SearchProvider searchProvider) {
        mExtractor = extractor;
        mSearchProvider = searchProvider;
        jCheckboxShowContextActionPerformed(null);
    }
    
    public interface SearchProvider {
        void find(String searchString, boolean useTaskContext);
    }
        

    /**
     * This method is called from within the constructor to initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is always
     * regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {
        java.awt.GridBagConstraints gridBagConstraints;

        jScrollPane2 = new javax.swing.JScrollPane();
        jTable1 = new javax.swing.JTable();
        jCheckboxShowContext = new javax.swing.JCheckBox();
        jButton1 = new javax.swing.JButton();

        setLayout(new java.awt.GridBagLayout());

        jTable1.setAutoCreateRowSorter(true);
        jTable1.setModel(new javax.swing.table.DefaultTableModel(
            new Object [][] {

            },
            new String [] {
                "Type", "Value"
            }
        ) {
            boolean[] canEdit = new boolean [] {
                false, false
            };

            public boolean isCellEditable(int rowIndex, int columnIndex) {
                return canEdit [columnIndex];
            }
        });
        jTable1.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseClicked(java.awt.event.MouseEvent evt) {
                jTable1MouseClicked(evt);
            }
        });
        jScrollPane2.setViewportView(jTable1);

        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 1;
        gridBagConstraints.gridwidth = 2;
        gridBagConstraints.fill = java.awt.GridBagConstraints.BOTH;
        gridBagConstraints.weightx = 1.0;
        gridBagConstraints.weighty = 1.0;
        add(jScrollPane2, gridBagConstraints);

        jCheckboxShowContext.setText("Show Task Context");
        jCheckboxShowContext.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jCheckboxShowContextActionPerformed(evt);
            }
        });
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 0;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.NORTHWEST;
        add(jCheckboxShowContext, gridBagConstraints);

        jButton1.setText("Wrte CSV");
        jButton1.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButton1ActionPerformed(evt);
            }
        });
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 1;
        gridBagConstraints.gridy = 0;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
        add(jButton1, gridBagConstraints);
    }// </editor-fold>//GEN-END:initComponents

    private void jTable1MouseClicked(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_jTable1MouseClicked
        if(evt.getClickCount()==2) {
            mSearchProvider.find(jTable1.getModel().
                    getValueAt(jTable1.getSelectedRow(), 0).toString(), 
                    jCheckboxShowContext.isSelected());
        }
    }//GEN-LAST:event_jTable1MouseClicked

    private void jCheckboxShowContextActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jCheckboxShowContextActionPerformed
        DefaultTableModel model = (DefaultTableModel) jTable1.getModel();
        model.setDataVector(new Object[][]{}, 
                new Object[]{model.getColumnName(0), model.getColumnName(1)});
        for(Map.Entry<String, Integer> pair : mExtractor.getClassCount(
                jCheckboxShowContext.isSelected()).entrySet()) {
            Object data[] = new Object[]{pair.getKey(), pair.getValue()};
            model.addRow(data);
        }
    }//GEN-LAST:event_jCheckboxShowContextActionPerformed

    private void jButton1ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButton1ActionPerformed
        JFileChooser fileChooser = new JFileChooser();
        fileChooser.setFileSelectionMode(JFileChooser.FILES_ONLY);
        fileChooser.setFileFilter(new FileFilter() {
            @Override
            public boolean accept(File file) {
                return file.getName().endsWith("csv") || file.isDirectory();
            }
            @Override
            public String getDescription() {
                return "CSV Files";
            }
        });
        try {
            if(fileChooser.showSaveDialog(fileChooser)==
                    JFileChooser.APPROVE_OPTION) {
                File csvFile = fileChooser.getSelectedFile();
                FileOutputStream stream = new FileOutputStream(csvFile);
                OutputStreamWriter writer = new OutputStreamWriter(stream);
                for(Object line : ((DefaultTableModel) jTable1.getModel()).
                        getDataVector()) {                    
                    for(Object column : (Vector) line) {
                        writer.write(column.toString() + ";");                        
                    }
                    writer.write("\n");
                }
                writer.close();
                stream.close();
            }            
        } catch(Exception e) {
            JOptionPane.showMessageDialog(null, e.getMessage(), "Error", 
                    JOptionPane.ERROR_MESSAGE);
        }

    }//GEN-LAST:event_jButton1ActionPerformed

    private OWLExtractor mExtractor;
    private SearchProvider mSearchProvider;
    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JButton jButton1;
    private javax.swing.JCheckBox jCheckboxShowContext;
    private javax.swing.JScrollPane jScrollPane2;
    private javax.swing.JTable jTable1;
    // End of variables declaration//GEN-END:variables
}
