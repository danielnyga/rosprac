(in-package :cram-robot-memory)

(defun complete()
  (let ((config	(cram-rosmln:make-simple-config "learnt_mlns/task_and_designator.mln"
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
