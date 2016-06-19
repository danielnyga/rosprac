(in-package :cram-robot-memory)

(defconstant prob-threshold 0.00001)

(defun complete-with-next-on-failure(do-sth-with designator &optional task goal-ctxt goal-parm)
  (car (with-completed-designator
         #'(lambda(d)
           (cram-language:with-failure-handling
               ((cram-language:plan-failure (e)
                  (roslisp:ros-info (cram-robot-memory-demo)
                                    "encountered an ~a error - retrying..." e)
                  (return nil)))
             `(,(funcall do-sth-with d))))
         designator task goal-ctxt goal-parm)))                       

(defun with-completed-designator(do-sth-with designator &optional task goal-ctxt goal-parm) 
  (let*((use-simplified-mln (and (null task) (null goal-ctxt) (null goal-parm)))
        (config (cram-rosmln:make-simple-config (if use-simplified-mln
                                                    "../learnt_mlns/designator.mln"
                                                    "../learnt_mlns/task_and_designator.mln")
                                                :fast-exact
                                                :first-order-logic
                                                :prac-grammar))
        (properties (get-designator-properties designator ""))
        (evidence (convert-to-evidence properties
                                       (not use-simplified-mln) task goal-ctxt goal-parm)))
    (roslisp:ros-info (cram-robot-memory) "completing designator ~a..." designator)
    (completion-result-target-fun-result
     (complete-key (make-completion-args :rosmln-config config
                                         :designator-properties properties
                                         :evidence evidence
                                         :seen-keys nil
                                         :tried-properties nil
                                         :target-function do-sth-with)))))

(defun test-completion() ;This is just for testing purposes - it can be removed...
	(let*((mug (cram-designators:make-designator :object `((:name "mug")))))
    (print (with-completed-designator #'(lambda(x) (declare (ignore x)) nil) mug)); 'achieve "OBJECT-IN-HAND ?OBJ"))
    t))

(defstruct completion-args
  rosmln-config
  designator-properties
  evidence
  seen-keys
  tried-properties
  target-function)

(defun update-completion-args
  (comp-args &key (rosmln-config (completion-args-rosmln-config comp-args))
                  (designator-properties (completion-args-designator-properties comp-args))
                  (evidence (completion-args-evidence comp-args))
                  (seen-keys (completion-args-seen-keys comp-args))
                  (tried-properties (completion-args-tried-properties comp-args))
                  (target-function (completion-args-target-function comp-args)))
  (make-completion-args :rosmln-config rosmln-config
                        :designator-properties designator-properties
                        :evidence evidence
                        :seen-keys seen-keys
                        :tried-properties tried-properties
                        :target-function target-function))

(defstruct completion-result
  (target-fun-result nil)
  (tried-properties nil)
  (call-in-last-rec nil))

(defun update-completion-result
  (comp-result &key (target-fun-result (completion-result-target-fun-result comp-result))
                    (tried-properties (completion-result-tried-properties comp-result))
                    (call-in-last-rec (completion-result-call-in-last-rec comp-result)))
  (make-completion-result :target-fun-result target-fun-result
                          :tried-properties tried-properties
                          :call-in-last-rec call-in-last-rec))

(defun complete-key(comp-args)
  (let*((prop-idx (length (completion-args-designator-properties comp-args)))
        (prop-atom (format nil "designatorProperty(Prop~D, Designator)" prop-idx))
        (evidence (cons prop-atom (completion-args-evidence comp-args)))
        (query (format nil "propertyKey(Prop~D, ?k)" prop-idx))
        (result (cram-rosmln:evidence-query (completion-args-rosmln-config comp-args)
                                            query
                                            evidence)))
    (complete-key-rec (update-completion-args comp-args :evidence evidence) result)))

(defun complete-key-rec(comp-args results)
  (let*((tried-props (completion-args-tried-properties comp-args))
        (desig-props (completion-args-designator-properties comp-args))
        (target-func (completion-args-target-function comp-args))
        (seen-keys (completion-args-seen-keys comp-args))
        (best-key (get-argument-from-best-result results 1))
        (best-prob (car (car results)))
        (uniform-key-dist (every #'(lambda(e)(= (car e) best-prob)) results))
        (no-more-keys (or (< best-prob prob-threshold) uniform-key-dist)))
    (cond ((and no-more-keys (already-tried comp-args))
           (make-completion-result :tried-properties tried-props))
          (no-more-keys
           (roslisp:ros-info (cram-robot-memory) "completed ~a" desig-props)
           (make-completion-result :target-fun-result
                                   (funcall target-func (get-designator desig-props))
                                   :tried-properties (cons desig-props tried-props)))
          ((member-if #'(lambda(x) (equal x best-key)) seen-keys)
           (complete-key-rec comp-args (cdr results)))
          (t
           (recurse-if-result-null
            (complete-value comp-args best-key)
            #'(lambda(result)
                (if (completion-result-call-in-last-rec result) ;retuned from a direct call
                    (update-completion-result result :call-in-last-rec nil)
                    (complete-key-rec
                     (update-completion-args
                      comp-args
                      :tried-properties (completion-result-tried-properties result))
                     (cdr results)))))))))

(defun complete-value(comp-args key)
  (if (already-tried comp-args key)
      (make-completion-result :tried-properties (completion-args-tried-properties comp-args))
      (let*((prop-idx (length (completion-args-designator-properties comp-args)))
            (best-key-atom (format nil "propertyKey(Prop~D,~S)" prop-idx key))
            (evidence (cons best-key-atom (completion-args-evidence comp-args)))
            (query (format nil "propertyValue(Prop~D, ?v)" prop-idx))
            (conf (completion-args-rosmln-config comp-args))
            (result (cram-rosmln:evidence-query conf query evidence))
            (second-best-prob (car (car (cdr result))))
            (seen-keys (completion-args-seen-keys comp-args))
            (seen-upd (if (< second-best-prob prob-threshold)
                          (cons key seen-keys)
                          seen-keys)))
        (complete-val-rec (update-completion-args comp-args
                                                  :evidence evidence
                                                  :seen-keys seen-upd)
                          result key))))

(defun complete-val-rec(comp-args results key)
  (let*((desig-props (completion-args-designator-properties comp-args))
        (evidence (completion-args-evidence comp-args))
        (tried-props (completion-args-tried-properties comp-args))
        (target-func (completion-args-target-function comp-args))
        (seen-keys (completion-args-seen-keys comp-args))
        (best-val (get-argument-from-best-result results 1))
        (prop-idx (length desig-props))
        (best-prob (car (car results)))
        (uniform-val-dist (every #'(lambda(e)(= (car e) best-prob)) results))
        (kv-pair `(,key ,best-val))
        (updated-props (cons kv-pair desig-props))
        (updated-ev (cons (format nil "propertyValue(Prop~D,~S)" prop-idx best-val) evidence))
        (no-more-values (or (< best-prob prob-threshold) uniform-val-dist)))
    (cond ((and no-more-values (already-tried comp-args))
           (make-completion-result :tried-properties tried-props))
          (no-more-values
           (roslisp:ros-info (cram-robot-memory) "completed ~a" desig-props)
           (make-completion-result :target-fun-result
                                   (funcall target-func (get-designator desig-props))
                                   :tried-properties (cons desig-props tried-props)
                                   :call-in-last-rec t))
          ((member-if #'(lambda(x) (equal x kv-pair)) desig-props)
           (complete-val-rec comp-args (cdr results) key))
          ((equal best-val "<hard coded pose>")
           (make-completion-result :tried-properties tried-props))
          (t
           (roslisp:ros-info (cram-robot-memory) "updated: ~a" updated-props)
           (recurse-if-result-null
            (complete-key (update-completion-args comp-args
                                                  :designator-properties updated-props
                                                  :evidence updated-ev
                                                  :seen-keys (cons key seen-keys)))
            #'(lambda(result)
                (complete-val-rec
                 (update-completion-args
                  comp-args
                  :tried-properties (completion-result-tried-properties result))
                 (cdr results) key)))))))

(defun already-tried(comp-args &optional key)
  (let*((tried-props (completion-args-tried-properties comp-args))
        (desig-props (completion-args-designator-properties comp-args))
        (tries (remove-if (lambda(w) (not (subsetp desig-props w :test #'equal))) tried-props)))
    (if (null key)
        (not (null tries))
        (some #'(lambda(w) (some #'(lambda(kv) (equal (car kv) key)) w)) tries))))

(defun get-designator-type(designator)
  (let* ((class-name (symbol-name (type-of designator)))
         (designator-name (subseq class-name 0 (search "-DESIGNATOR" class-name))))
    (string-downcase designator-name)))

(defun get-designator(designator-properties)
  (labels((split-recursively(rest pattern to-return)
            (let((pos (search pattern rest)))
              (if (null pos)
                  (concatenate 'list to-return `(,rest))
                  (split-recursively (subseq rest (+ pos (length pattern)))
                                     pattern
                                     (concatenate 'list to-return `(,(subseq rest 0 pos)))))))
          (split-keys(properties)
            (map 'list
                 #'(lambda(kv) (cons (split-recursively (car kv) "::" nil) (cdr kv)))
                 properties))
          (group-recursively(rest current-key current-list return-list)
            (let*((pair (car rest))
                  (key (car pair))
                  (key-part (car key))
                  (value (car (last pair)))
                  (rest-pair `(,(cdr key) ,value)))
              (cond ((null pair) (cons `(,current-key ,(make-desig-or-val(group current-list)))
                                       return-list))     
                    ((null key-part) value)
                    ((not (equal key-part current-key))
                     (group-recursively (cdr rest) key-part `(,rest-pair)
                                        (cons `(,current-key ,
                                                (make-desig-or-val (group current-list)))
                                              return-list)))
                    (t (group-recursively (cdr rest) key-part
                                          (cons rest-pair current-list) return-list)))))
          (group(splitted-desig-list)
            (let ((sorted (sort splitted-desig-list
                                #'string-lessp :key #'(lambda(x) (car (car x))))))
              (group-recursively sorted (car (car (car sorted))) nil nil)))
          (make-desig-or-val(kv-pairs-or-value)
            (if (not (typep kv-pairs-or-value 'string))
                (make-designator kv-pairs-or-value)
                (let ((pose (string-to-pose kv-pairs-or-value)))
                  (cond ((not (null pose)) pose)
                        ((every #'identity (map 'list #'upper-case-p kv-pairs-or-value))
                         (to-keyword kv-pairs-or-value))
                        (t kv-pairs-or-value)))))
          (remove-desig-type-convert-to-keywd(kv)
            (cons (to-keyword (car (cdr (split-recursively (car kv) "." nil)))) (cdr kv)))
          (make-designator(key-value-pairs)
            (let*((desig-type (car (split-recursively (car (car key-value-pairs)) "." nil)))
                  (kv-pairs (map 'list #'remove-desig-type-convert-to-keywd key-value-pairs)))
              (cram-designators:make-designator (to-keyword desig-type) kv-pairs)))
          (to-keyword(str) (symbol-value (intern (string-upcase str) "KEYWORD"))))
    (make-desig-or-val (group (split-keys designator-properties)))))

(defun convert-to-evidence(properties include-task-info
                           &optional task goal-context goal-param )
  (concatenate 'list
               (properties-to-evidence properties "Designator"
                                       (if include-task-info "Task" nil)
                                       goal-param)
               (if include-task-info 
                   (task-information-to-evidence "Task" t
                                                 (if (null task) nil
                                                     (symbol-name task))
                                                 goal-context)
                   nil)))

(defun properties-to-evidence(properties desig-id
                              &optional task-id goal-param)
  (concatenate
   'list
   (flatten-list
    (enumerate-and-map-list
     properties
     0
     #'(lambda(index key-value-pair)
         (let* ((key (car key-value-pair))
                (value (car (last key-value-pair)))
                (propAtom (format nil "designatorProperty(Prop~D,~a)" index desig-id))
                (keyAtom (format nil "propertyKey(Prop~D,~S)" index key))
                (valueAtom (format nil "propertyValue(Prop~D,~S)" index value)))
           `(,propAtom ,keyAtom ,valueAtom)))) nil)
   (if (null task-id) nil `(,(format nil "goalParameter(~a,~a)" desig-id task-id)))
   (if (null goal-param)
       nil
       `(,(format nil "goalParameterKey(~a,~S)" desig-id goal-param))))) 

(defun task-information-to-evidence(task-id success &optional task-name goal-context)
  (concatenate
   'list
   (if success `(, (format nil "success(~a)" task-id)) nil)
   (if (null task-name) nil `(,(format nil "name(~a,~S)" task-id task-name)))
   (if (null goal-context)
       nil
       `(,(format nil "goalPattern(~a,~S)" task-id
                 (if (equal goal-context "") " " goal-context)))))) 


(defun get-designator-properties(designator prefix)
  (flatten-list
   (map 'list
        (lambda(element)
          (let* ((key-suffix (symbol-name (car element)))
                 (desig-type (get-designator-type designator))
                 (key (if (= (length prefix) 0)
                          (format nil "~a.~a" desig-type key-suffix)
                          (format nil "~a::~a.~a" prefix desig-type key-suffix)))
                 (value (car (cdr element))))
            (typecase value
              (symbol (cons `(,key ,(symbol-name value)) nil))
              (string (cons `(,key ,value) nil))
              (float (cons `(,key ,(format nil "~,3f" value)) nil))
              (cram-designators:designator (get-designator-properties value key))
              (cl-transforms-stamped:pose (cons `(,key ,(pose-to-string value)) nil)))))
        (cram-designators:description designator))
   nil))

(defun enumerate-and-map-list(lst idx func)
  (if (null lst)
      nil
      (cons (funcall func idx (car lst)) (enumerate-and-map-list (cdr lst) (+ idx 1) func))))

(defun flatten-list(lst result)
  (if (null lst)
      result
      (flatten-list (cdr lst) (concatenate 'list (car lst) result))))

(defun pose-to-string(pose)
  (let*((position (cl-transforms:origin pose))
        (orientation (cl-transforms:orientation pose))
        (pos-x (cl-transforms:x position))
        (pos-y (cl-transforms:y position))
        (pos-z (cl-transforms:z position))
        (or-x (cl-transforms:x orientation))
        (or-y (cl-transforms:y orientation))
        (or-z (cl-transforms:z orientation))
        (or-w (cl-transforms:w orientation))
        (frame (if (typep pose 'cl-transforms-stamped:pose-stamped)
                 (cl-transforms-stamped:frame-id pose)
                 nil)))
       (if (null frame)
           (format nil "(~,3f,~,3f,~,3f,~,3f,~,3f,~,3f,~,3f)"
                   pos-x pos-y pos-z or-w or-x or-y or-z)
           (format nil "(~a,~,3f,~,3f,~,3f,~,3f,~,3f,~,3f,~,3f)"
                   frame pos-x pos-y pos-z or-w or-x or-y or-z))))

(defun string-to-pose(pose-string);Unfortunately, lisp has no built-in support for regex...
  (labels((parse(str current-word to-return)
            (cond ((null str) (reverse (cons current-word to-return)))
                  ((equal (car str) #\,) (parse (cdr str) nil (cons current-word to-return)))
                  (t (parse (cdr str) (cons (car str) current-word) to-return)))))
    (let*((parts (if (and (equal (char pose-string 0) #\()
                          (equal (char pose-string (- (length pose-string) 1)) #\)))
                     (parse (map 'list #'identity
                                 (subseq pose-string 1 (- (length pose-string) 1))) nil nil)
                     nil))
          (str-parts (map 'list #'(lambda(e) (map 'string #'identity (reverse e))) parts))
          (pose (case (length str-parts) 
                  (7 str-parts)
                  (8 (subseq str-parts 1))
                  (otherwise nil)))
          (floats (map 'list #'(lambda(s) (with-input-from-string (i s) (read i))) pose))
          ;TODO: The previous line is a potential security hole...
          (list-is-pose (and (= (length floats) 7)
                             (every #'(lambda(e) (typep e 'float)) floats)))
          (is-pose (and list-is-pose (= (length str-parts) 7)))
          (is-pose-stamped (and list-is-pose
                                (= (length str-parts) 8)
                                (typep (car str-parts) 'string)))
          (position (if (not list-is-pose) nil (cl-transforms:make-3d-vector (nth 0 floats)
                                                               (nth 1 floats)(nth 2 floats))))
          (orientation (if (not list-is-pose) nil
                           (cl-transforms:normalize (cl-transforms:make-quaternion
                                                     (nth 4 floats) (nth 5 floats)
                                                     (nth 6 floats) (nth 3 floats))))))
      (cond (is-pose (cl-transforms:make-pose position orientation))
            (is-pose-stamped (cl-transforms-stamped:make-pose-stamped
                              (car str-parts)
                              0.0
                              position
                              orientation))
            (t nil)))))
                                                                      
(defun get-argument-from-best-result(results argument-index)
  (let*((best-result (cram-rosmln:split-atom (car (last (car results)))))
        (argument (nth (+ 1 argument-index) best-result)))
    (if (> (length argument) 2)
        (subseq argument 1 (- (length argument) 1))
        "")))

(defun recurse-if-result-null(result recursion-function)
  (if (null (completion-result-target-fun-result result))
      (funcall recursion-function result)
      result))
