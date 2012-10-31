;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Script to non-interactively recompile all byte-compiled elisp files (.elc's)
;; for which an associated source (.el) file is present.  This is intended to
;; be run by root in a batch fashion:
;;
;;   % emacs --batch --load byte-compile-all-elc-files.el
;;
;; The script exits when done
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(require 'cl)
(loop for directory in load-path do 
     (if (file-exists-p directory)
	 (loop for file in
	      (directory-files directory t "\\.elc$") do
	      (let ((source-file (replace-regexp-in-string "\\.elc" ".el" file)))
		(if (file-exists-p source-file)
		    (byte-compile-file source-file))))))
(kill-emacs)
