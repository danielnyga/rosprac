(in-package :cram-robot-memory)

(defconstant prob-threshold 0.00001)

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
    (car (complete-key config properties evidence nil nil do-sth-with))))

(defun test-completion() ;This is just for testing purposes - it can be removed...
	(let*((mug (cram-designators:make-designator :object `((:name "mug")))))
    (print (with-completed-designator #'(lambda(x) (declare (ignore x)) nil) mug)); 'achieve "OBJECT-IN-HAND ?OBJ"))
    t))

(defun complete-key(conf desig-props property-evidence seen-keys tried-props target-func)
  (let*((prop-idx (length desig-props))
        (prop-atom (format nil "designatorProperty(Prop~D, Designator)" prop-idx))
        (evidence (cons prop-atom property-evidence))
        (query (format nil "propertyKey(Prop~D, ?k)" prop-idx))
        (result (cram-rosmln:evidence-query conf query evidence)))
    (complete-key-rec conf desig-props evidence seen-keys tried-props target-func result nil)))

(defun complete-key-rec(conf desig-props evidence seen-keys tried-props
                        target-func results called)
  (let*((best-key (get-argument-from-best-result results 1))
        (best-prob (car (car results)))
        (uniform-key-dist (every #'(lambda(e)(= (car e) best-prob)) results))
        (no-more-keys (or (< best-prob prob-threshold) uniform-key-dist)))
    (cond ((and no-more-keys called)
           `(nil ,tried-props nil))
          (no-more-keys
           (roslisp:ros-info (cram-robot-memory) "completed ~a" desig-props)
           `(,(funcall target-func (get-designator desig-props))
             ,(cons desig-props tried-props)
             nil))
          ((member-if #'(lambda(x) (equal x best-key)) seen-keys)
           (complete-key-rec conf desig-props evidence seen-keys
                             tried-props target-func (cdr results) called))
          (t
           (recurse-if-car-null (complete-value conf desig-props evidence
                                                seen-keys tried-props target-func best-key)
                                #'(lambda(result)
                                    (if (car (last result)) ;retuned from a direct call
                                        `(,(nth 0 result) ,(nth 1 result) nil)
                                        (complete-key-rec conf desig-props evidence seen-keys
                                                          (car (cdr result))
                                                          target-func (cdr results) t))))))))

(defun complete-value(conf desig-props key-evidence seen-keys tried-props target-func key)
  (let*((tries (remove-if (lambda(w) (not (subsetp desig-props w :test #'equal))) tried-props))
        (already-tried(some #'(lambda (w) (some #'(lambda (kv) (equal (car kv) key)) w)) tries)))
    (if already-tried
        `(nil ,tried-props nil)
        (let*((prop-idx (length desig-props))
              (best-key-atom (format nil "propertyKey(Prop~D,~S)" prop-idx key))
              (evidence (cons best-key-atom key-evidence))
              (query (format nil "propertyValue(Prop~D, ?v)" prop-idx))
              (result (cram-rosmln:evidence-query conf query evidence))
              (second-best-prob (car (car (cdr result))))
              (seen-upd (if (< second-best-prob prob-threshold) (cons key seen-keys) seen-keys)))
          (complete-val-rec conf desig-props evidence seen-upd
                            tried-props target-func result key nil)))))

(defun complete-val-rec(conf desig-props evidence seen-keys
                        tried-props target-func results key called)
  (let*((best-val (get-argument-from-best-result results 1))
        (prop-idx (length desig-props))
        (best-prob (car (car results)))
        (uniform-val-dist (every #'(lambda(e)(= (car e) best-prob)) results))
        (kv-pair `(,key ,best-val))
        (updated-props (cons kv-pair desig-props))
        (updated-ev (cons (format nil "propertyValue(Prop~D,~S)" prop-idx best-val) evidence))
        (no-more-values (or (< best-prob prob-threshold) uniform-val-dist)))
    (cond ((and no-more-values called)
           `(nil ,tried-props nil))
          (no-more-values
           (roslisp:ros-info (cram-robot-memory) "completed ~a" desig-props)
           `(,(funcall target-func (get-designator desig-props))
             ,(cons desig-props tried-props)
             t))
          ((member-if #'(lambda(x) (equal x kv-pair)) desig-props)
           (complete-val-rec conf desig-props evidence seen-keys
                             tried-props target-func (cdr results) key called))
          ((equal best-val "<hard coded pose>")
           `(nil ,tried-props nil))
          (t
           (roslisp:ros-info (cram-robot-memory) "updated: ~a" updated-props)
           (recurse-if-car-null (complete-key conf updated-props updated-ev
                                              (cons key seen-keys) tried-props target-func)
                                #'(lambda(result)
                                    (complete-val-rec conf desig-props evidence seen-keys
                                                      (car (cdr result))
                                                      target-func (cdr results) key t)))))))
    
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
                          (format nil "~a.~a::~a" desig-type prefix key-suffix)))
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

(defun recurse-if-car-null(result recursion-function)
  (if (null (car result))
      (funcall recursion-function result)
      result))
