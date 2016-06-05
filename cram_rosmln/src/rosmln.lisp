(in-package :cram-rosmln)

(defun make-simple-config(mln-file method logic grammar)
  (roslisp:make-msg "rosmln/MLNConfig"
                    (mlnFiles) mln-file
                    (db) ""
                    (method) (ecase method (:fast-exact "FastExact"))
                    (output_filename) ""
                    (saveResults) nil
                    (logic) (ecase logic (:first-order-logic "FirstOrderLogic") (:fuzzy-logic "FuzzyLogic"))
                    (grammar) (ecase grammar (:prac-grammar "PRACGrammar") (:standard-grammar "StandardGrammar"))))

(defun evidence-query(config query evidence)
  (let* ((results (roslisp:call-service "mln_interface" 'rosmln-srv:MLNInterface
                                        :query (roslisp:make-msg "rosmln/MLNQuery"
                                                                 (queries) query
                                                                 (evidence) (map 'vector #'identity evidence))
                                        :config config))
         (atomProbPairs (roslisp:with-fields ((evidence (evidence response))) results
                          (map 'list
                               #'(lambda(result) (roslisp:with-fields (atom prob) result `(,prob ,atom)))
                               evidence)))
         (sortedPairs (sort atomProbPairs #'> :key #'car)))
    sortedPairs))
