;; add emacs.d to load path
(add-to-list 'load-path "~/.emacs.d")


;; (defun system-type-is-darwin ()
;;   (interactive)
;;     "Return true if system is darwin-based (Mac OS X)"
;;       (string-equal system-type "darwin"))

;; (if (system-type-is-darwin)
;; (setq TeX-view-program-selection '((output-pdf "PDF Viewer"))))

;; (if (system-type-is-darwin)
;; (setq TeX-view-program-list
;;            '(("PDF Viewer" "/Applications/Skim.app/Contents/SharedSupport/displayline -b -g %n %o %b"))))
;; ;;(server-start); start emacs in server mode so that skim can talk to it

;; (setq ispell-dictionary "en")

;; ;;AucTEX
;; (setenv "PATH" (concat "/usr/texbin:/usr/local/bin:" (getenv "PATH")))
;; (setq exec-path (append '("/usr/texbin" "/usr/local/bin") exec-path))
;; (load "auctex.el" nil t t)
;; (load "preview-latex.el" nil t t)
;; (setq TeX-auto-save t)
;; (setq TeX-parse-self t)
;; ;;;;;;(setq-default TeX-master nil)
;; (setq-default TeX-master "master")
;; (setq TeX-PDF-mode t)
;; (defun guess-TeX-master (filename)
;;   "Guess the master file for FILENAME from currently open .tex files."
;;   (let ((candidate nil)
;; 	(filename (file-name-nondirectory filename)))
;;     (save-excursion
;;       (dolist (buffer (buffer-list))
;; 	(with-current-buffer buffer
;; 	  (let ((name (buffer-name))
;; 		(file buffer-file-name))
;; 	    (if (and file (string-match "\\.tex$" file))
;; 		(progn
;; 		  (goto-char (point-min))
;; 		  (if (re-search-forward (concat "\\\\input{" filename "}") nil t)
;; 		      (setq candidate file))
;; 		  (if (re-search-forward (concat "\\\\include{" (file-name-sans-extension filename) "}") nil t)
;; 		      (setq candidate file))))))))
;;     (if candidate
;; 	(message "TeX master document: %s" (file-name-nondirectory candidate)))
;;     candidate))

;; ;(setq TeX-master (guess-TeX-master (buffer-file-name)))




;;CEDET
;;(load-file "~/Dropbox/Homedir/env/cedet-1.1/common/cedet.el")
;; Enable EDE (Project Management) features
;;(global-ede-mode 1)
;; Enabling Semantic (code-parsing, smart completion) features
;; Select one of the following:

;; * This enables the database and idle reparse engines
;(semantic-load-enable-minimum-features)
;; * This enables some tools useful for coding, such as summary mode,
;;   imenu support, and the semantic navigator
;(semantic-load-enable-code-helpers)
;(global-srecode-minor-mode 1)            ; Enable template insertion menu
;(semantic-load-enable-gaudy-code-helpers)
;;(semantic-load-enable-semantic-debugging-helpers)
;;(setq-mode-local c-mode semanticdb-find-default-throttle '(project unloaded system recursive))

;; hide menu bar
(menu-bar-mode 0)
;;(tool-bar-mode 0)

(global-linum-mode t)

(setq linum-format "%d ")
(custom-set-variables 
'(column-number-mode t)
'(display-time-mode t)
'(fill-column 90002000)
'(inhibit-startup-screen t)
'(safe-local-variable-values (quote ((tex-pdf\? . t))))
'(show-paren-mode t)
'(text-mode-hook (quote (turn-on-auto-fill text-mode-hook-identify))))
;;; End Emacs' contributions

;; Cut/copy/paste in sync with other programs                
(global-set-key "\C-w" 'clipboard-kill-region)
(global-set-key "\M-w" 'clipboard-kill-ring-save)
(global-set-key "\C-y" 'clipboard-yank)
(global-set-key "\M-p" 'backward-paragraph)
(global-set-key "\M-n" 'forward-paragraph)

;;; "Dangerous functions"
(put 'narrow-to-region 'disabled nil)
(put 'upcase-region 'disabled nil)
(put 'downcase-region 'disabled nil)

;;; Unwanted functions
(put 'center-line 'disabled t)
(put 'center-paragraph 'disabled t)

;; move backup- and auto-save-files into temp folder
(setq backup-directory-alist
      `((".*" . ,temporary-file-directory)))
(setq auto-save-file-name-transforms
      `((".*" ,temporary-file-directory t)))

;; enable easy window switching
(require 'windmove)
(if (fboundp 'windmove-default-keybindings)
      (windmove-default-keybindings 'meta))

;; ido mode
(require 'ido)
(ido-mode t)

;; auto-complete mode
(require 'auto-complete-config)
(add-to-list 'ac-dictionary-directories "~/.emacs.d/ac-dict")
(ac-config-default)
(global-auto-complete-mode t)
(define-key ac-mode-map (kbd "M-TAB") 'auto-complete)
(add-hook 'c++-mode (lambda () (add-to-list 'ac-sources 'ac-source-semantic)))

;;latex auto-complete
(require 'auto-complete-latex)
(setq ac-l-dict-directory "~/.emacs.d/ac-l-dict/")
(add-to-list 'ac-modes 'foo-mode)
(add-hook 'foo-mode-hook 'ac-l-setup)

(require 'ac-math)
(add-to-list 'ac-modes 'latex-mode)   ; make auto-complete aware of {{{latex-mode}}}
(add-to-list 'ac-modes 'LaTeX-mode)   ; make auto-complete aware of {{{latex-mode}}}


(defun ac-latex-mode-setup ()         ; add ac-sources to default ac-sources
  (setq ac-sources
	(append '(ac-source-math-unicode ac-source-math-latex ac-source-latex-commands)
		ac-sources))
  )
(ac-flyspell-workaround)

(add-hook 'LaTeX-mode-hook 'ac-latex-mode-setup)

(add-hook 'LaTeX-mode-hook 'flyspell-mode)
(add-hook 'LaTeX-mode-hook 'TeX-source-correlate-mode)
(setq TeX-source-correlate-method 'synctex)

;;(setq ac-auto-start 3)


;; yaml-mode
(require 'yaml-mode)
(add-to-list 'auto-mode-alist '("\\.yml$" . yaml-mode))
(add-to-list 'auto-mode-alist '("\\.yaml$" . yaml-mode))

;; xml-mode for launch scripts
(add-to-list 'auto-mode-alist '("\\.launch$" . xml-mode))

;; winner-mode
(when (fboundp 'winner-mode)
  (winner-mode 1))


(global-set-key [(meta shift return)] 'switch-full-screen)
(global-set-key "\C-c\C-c" 'comment-region)
(global-set-key "\C-c\C-u" 'uncomment-region)
(global-set-key "\C-c\C-l" "\C-u79;\C-m") ; Cover line in comment marks.
;; Switch RET and LF -- Makes <return> indent automatically
(global-set-key [?\C-m] 'newline-and-indent)                                                           
(global-set-key [?\C-j] 'newline)                                                                      
           

;; Window resizing
(global-set-key (kbd "C-M-<left>") 'shrink-window-horizontally)
(global-set-key (kbd "C-M-<right>") 'enlarge-window-horizontally)
(global-set-key (kbd "C-M-<down>") 'shrink-window)
(global-set-key (kbd "C-M-<up>") 'enlarge-window)

;; compile
(setq compile-command "cd .. && make -k")
(global-set-key "\C-cv" 'next-error)
(global-set-key "\C-cc" 'compile-again)

(setq compilation-last-buffer nil)
(defun compile-again (pfx)
  """Run the same compile as the last time. If there was no last time, or there is a prefix argument, this acts like M-x compile."""
  (interactive "p")
  (if (and (eq pfx 1)
	   compilation-last-buffer)
      (progn
	(set-buffer compilation-last-buffer)
	(revert-buffer t t))
    (call-interactively 'compile)))

(setq compilation-finish-functions 'compile-autoclose)
(defun compile-autoclose (buffer string)
  (cond ((string-match "finished" string)
	 (bury-buffer "*compilation*")
	 (winner-undo)
	 (message "Build successful."))
	(t                                                                    
	 (message "Compilation exited abnormally: %s" string)
	 (next-error)
	 )))
(put 'downcase-region 'disabled nil)

;;
