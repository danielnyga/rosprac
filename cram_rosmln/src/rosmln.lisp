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
                               evidence))))
    (print query)
    (print evidence)
    (if (null atomProbPairs)
        (cram-language:fail 'mln-query-failure)
        atomProbPairs)))

(define-condition mln-query-failure (cram-language:plan-failure) ())
                   
(defun split-atom(atom)
  (let* ((first-parenthesis (position #\( atom))
         (predicate (subseq atom first-parenthesis))
         (arguments (subseq atom (+ first-parenthesis 1) (- (length atom) 1))))
    (labels
        ((split-recursively(string current-word ret quoted)
           (if (null string)
               (cons current-word ret)
               (case (car string)
                 (#\" (split-recursively(cdr string) (cons (car string) current-word) ret (not quoted)))
                 (#\, (if quoted
                          (split-recursively(cdr string) (cons (car string) current-word) ret quoted)
                          (split-recursively(cdr string) nil (cons current-word ret) quoted)))
                 (otherwise (split-recursively(cdr string) (cons (car string) current-word) ret quoted)))))
         (split-arguments()
           (map 'list
                #'(lambda(word) (map 'string #'identity (reverse word)))
                (reverse (split-recursively (map 'list #'identity arguments) nil nil nil)))))
      (cons predicate (split-arguments)))))
                
         
             
