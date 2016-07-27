(in-package :cram-rosmln)

(defun make-simple-config(mln-file method logic grammar)
  "This function creates a MLN configuration stating how a MLN shall be queried.
   It is intended to be used with the evidence-query function.
   mln-file is a path in form of a string specifying the MLN to be queried relative to the cwd of the rosmln service.
   method is the inference method to be used, which can be one of :fast-exact :gibbs-sampling :enumeration-ask :wcsp :sa-max-walk-sat
   logic is the logic to use, it can be either :first-order-logic or :fuzzy-logic
   grammar is the grammar to be used, it can be either :prac-grammar or :standard-grammar"
   
  (roslisp:make-msg "rosmln/MLNConfig"
                    (mlnFiles) mln-file
                    (db) ""
                    (method) (ecase method (:fast-exact "FastExact")
                                           (:gibbs-sampling "GibbsSampler")
                                           (:mc-sat "MCSAT")
                                           (:enumeration-ask "EnumerationAsk")
                                           (:wcsp "WCSPInference")
                                           (:sa-max-walk-sat "SAMaxWalkSAT"))
                    (output_filename) ""
                    (saveResults) nil
                    (logic) (ecase logic (:first-order-logic "FirstOrderLogic") (:fuzzy-logic "FuzzyLogic"))
                    (grammar) (ecase grammar (:prac-grammar "PRACGrammar") (:standard-grammar "StandardGrammar"))))


(defun evidence-query(config query evidence)
  "This function executes a query on the MLN in config given some evidence.
   config is a configuration created with make-simple-config.
   query is a query in form of a stirng.
   evidence is a list of strings, one string for each line in the evidence database.
   The return value is a list of lists, each sub list consists of a probability and a query."
  (let* ((service-name "mln_interface")
         (results
           (cond
             ((not (wait-for-service service-name 30))
              (roslisp:ros-error (cram-rosmln) "timeout while waiting for service ~a!" service-name)
              (cram-language:fail 'mln-query-failure))
             (t (roslisp:call-service service-name 'rosmln-srv:MLNInterface
                                      :query (roslisp:make-msg "rosmln/MLNQuery"
                                                               (queries) query
                                                               (evidence) (map 'vector #'identity evidence))
                                      :config config))))
         (atomProbPairs (roslisp:with-fields ((evidence (evidence response))) results
                          (map 'list
                               #'(lambda(result) (roslisp:with-fields (atom prob) result `(,prob ,atom)))
                               evidence))))
    (if (null atomProbPairs)
        (cram-language:fail 'mln-query-failure)
        atomProbPairs)))

(define-condition mln-query-failure (simple-condition) ())
                   
(defun split-atom(atom)
  "This function can be executed to split a ground atom which is the result of a query.
   For example, foo(bar, test) gets (foo, bar, test)."
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
                
         
             
