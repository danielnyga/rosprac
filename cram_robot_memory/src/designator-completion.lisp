(in-package :cram-robot-memory)

(defun test-completion()
	(let*((mug (cram-designators:make-designator
              :object `((:name "mug")))))
       (print (complete mug))
       t))

(defun complete(designator)
  (labels ((complete-recursively(properties)
             (let*((config	(cram-rosmln:make-simple-config "learnt_mlns/task_and_designator.mln"
                                                            :fast-exact
                                                            :first-order-logic
                                                            :prac-grammar))
                   (property-evidence (properties-to-evidence properties 0))
                   (designator-type (get-designator-type properties))
                   (dtype (format nil "designatorType(Designator,~S)" designator-type))
                   (prop-idx (length properties))                       
                   (prop-atom (format nil "designatorProperty(Prop~D, Designator)" prop-idx))
                   (other-evidence (cons prop-atom (cons dtype `("success(Task)"))))
                   (key-evidence (concatenate 'list property-evidence other-evidence))
                   (key-query (format nil "propertyKey(Prop~D, ?k)" prop-idx))
                   (key-result (cram-rosmln:evidence-query config key-query key-evidence))
                   (best-key (get-argument-from-best-result key-result 1))
                   ;TODO: Avoid duplicate keys + fixed poses...
                   (best-key-prob (car (car key-result)))
                   (uniform-key-dist (every #'(lambda(e)(= (car e) best-key-prob)) key-result))
                   (best-key-atom (format nil "propertyKey(Prop~D,~S)" prop-idx best-key))
                   (value-evidence (cons best-key-atom key-evidence))
                   (value-query (format nil "propertyValue(Prop~D, ?v)" prop-idx))
                   (val-result (cram-rosmln:evidence-query config value-query value-evidence))
                   (best-val (get-argument-from-best-result val-result 1))
                   (best-val-prob (car (car val-result)))
                   (uniform-val-dist (every #'(lambda(e)(= (car e) best-val-prob)) val-result)))               
               (if (or (< best-key-prob 0.000001)
                       (< best-val-prob 0.000001)
                       uniform-key-dist
                       uniform-val-dist)
                   properties
                   (complete-recursively (cons `(,best-key,best-val) properties))))))
    (get-designator (complete-recursively (get-designator-properties designator "")))))

(defun get-designator-type(designator)
  "object"
  ;TODO
)

(defun get-designator(designator-properties)
        
  ;TODO
)

(defun properties-to-evidence(designator-properties start-property-index)
  (flatten-list
   (enumerate-and-map-list
    designator-properties
    start-property-index
    #'(lambda(index key-value-pair)
        (let* ((key (car key-value-pair))
               (value (car (last key-value-pair)))
               (propertyAtom (format nil "designatorProperty(Prop~D,Designator)" index))
               (keyAtom (format nil "propertyKey(Prop~D,~S)" index key))
               (valueAtom (format nil "propertyValue(Prop~D,~S)" index value)))
          `(,propertyAtom ,keyAtom ,valueAtom)))) nil))

(defun get-designator-properties(designator prefix)
  (flatten-list
   (map 'list
        (lambda(element)
          (let* ((key-suffix (symbol-name (car element)))
                 (key (if (= (length prefix) 0)
                          key-suffix
                          (format nil "~a::~a" prefix key-suffix)))
                 (value (car (cdr element))))
            (typecase value
              (symbol (cons `(,key ,(concatenate 'string ":" (symbol-name value))) nil))
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

(defun get-argument-from-best-result(results argument-index)
  (let*((best-result (cram-rosmln:split-atom (car (last (car results)))))
        (argument (nth (+ 1 argument-index) best-result)))
    (if (> (length argument) 2)
        (subseq argument 1 (- (length argument) 1))
        "")))

(defun complete3()
  (let*((config	(cram-rosmln:make-simple-config "learnt_mlns/task_and_designator.mln"
                                                :fast-exact
                                                :first-order-logic
                                                :prac-grammar))
        (query "propertyKey(OtherProp, ?k)")
        (evidence '("success(Task)"
                    "designatorType(Designator, \"object\")"
                    "propertyKey(NameProp,\"NAME\")"
                    "propertyValue(NameProp, \"mug\")"
                    "designatorProperty(NameProp, Designator)"
                    "designatorProperty(OtherProp, Designator)")))
        (cram-rosmln:evidence-query config query evidence)))
