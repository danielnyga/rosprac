(in-package :cram-robot-memory)

(defun split-recursively(rest pattern &optional to-return)
            (let((pos (search pattern rest)))
              (if (null pos)
                  (concatenate 'list to-return `(,rest))
                  (split-recursively (subseq rest (+ pos (length pattern)))
                                     pattern
                                     (concatenate 'list to-return `(,(subseq rest 0 pos)))))))

(defun get-designator-type(designator)
  (let* ((class-name (symbol-name (type-of designator)))
         (designator-name (subseq class-name 0 (search "-DESIGNATOR" class-name))))
    (string-downcase designator-name)))

(defun get-designator(designator-properties)
  (labels((split-keys(properties)
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

(defun properties-to-evidence(properties desig-id start-index
                              &optional task-id goal-param)
  (concatenate
   'list
   (flatten-list
    (enumerate-and-map-list
     properties
     start-index
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

(defun task-information-to-evidence(task-id &key include-failure failure task-name goal-context)
  (concatenate
   'list
   (if include-failure
       `(,(format nil "failure(~a,~S)" task-id (if (null failure) " " failure)))
       nil)
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

(defun get-rosmln-config(use-simplified-mln)
  (cram-rosmln:make-simple-config (if use-simplified-mln
                                      "../learnt_mlns/designator.mln"
                                      "../learnt_mlns/task_and_designator.mln")
                                  :fast-exact
                                  :first-order-logic
                                  :prac-grammar))
