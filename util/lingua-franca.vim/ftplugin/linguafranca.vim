" ftplugin/linguafranca.vim
" Arbitrary linguafranca related vim code

setlocal commentstring=//%s
setlocal formatoptions=jcroq
" 'comments' configuration stolen from the C file, let's see how it works
setlocal comments=sO:*\ -,mO:*\ \ ,exO:*/,s1:/*,mb:*,ex:*/,://

" function! CursorInTargetBlock()
"   " TODO
"   " See: :help searchpair(), searchpairpos(), `n` flag
" endfunction

" function! LFTargetTextObject()
"   " TODO
"   if ! CursorInTargetBlock()
"     return
"   endif
" endfunction

" Text objects for the target blocks
" Don't work very well but can be convenient
onoremap <buffer> i= :<C-u>execute "normal! ?{=?e+1\rv/=}/b-1\r"<cr>
xnoremap <buffer> i= :<C-u>execute "normal! ?{=?e+1\rv/=}/b-1\r"<cr>
onoremap <buffer> a= :<C-u>execute "normal! ?{=\rv/=}/e\r"<cr>
xnoremap <buffer> a= :<C-u>execute "normal! ?{=\rv/=}/e\r"<cr>
