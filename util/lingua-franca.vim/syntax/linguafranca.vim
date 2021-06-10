" syntax/linguafranca.vim

" quit when a syntax file was already loaded
if exists("b:current_syntax")
  finish
endif

function! LFGetTarget()
  " cf https://github.com/icyphy/lingua-franca/blob/0870cf86185733180d7b09f32fa5221a948af83b/org.lflang/src/org/lflang/LinguaFranca.xtext#L118
  " and https://www.eclipse.org/Xtext/documentation/301_grammarlanguage.html#common-terminals
  let l:pattern = '^\s*target\s*\(\h\h*\)\s*\({\|;\)\?\s*'
  for l in getbufline("", 1, "$")
    if l =~# l:pattern
      let l:result = substitute(l, l:pattern, '\1', "")
      break
    endif
  endfor

  if exists("l:result")
    return l:result
  else
    " TODO Handle error
    return "Cpp"
  endif
endfunction

function! LFGetSyntaxFile()
  return "syntax/". tolower(LFGetTarget()). ".vim"
endfunction

" case sensitive
syntax case match

" Keywords
syntax keyword lfKeywords target import main realtime reactor state time mutable input output timer
      \ action reaction startup shutdown after deadline mutation preamble new federated at as from
syntax keyword lfActionOrigins logical physical
syntax keyword lfTimeUnits nsec nsecs usec usecs msec msecs sec secs second seconds
      \ min mins minute minutes hour hours day days week weeks

highlight def link lfKeywords Keyword
highlight def link lfActionOrigins Keyword
highlight def link lfTimeUnits StorageClass

" Matches
syntax match lfComment :\(#.*$\|//.*$\):
hi def link lfComment Comment
syntax match lfTargetDelim :\({=\|=}\):
hi def link lfTargetDelim Delimiter

" Regions
execute "syntax include @TARGET ". LFGetSyntaxFile()
syntax region lfTargetLang keepend start=/{=/ contains=@TARGET end=/=}/

syntax region lfBlockComment start=:/\*: end=:\*/:
hi def link lfBlockComment Comment

syntax region lfString start=:": skip=:\\": end=:":
hi def link lfString String

let b:current_syntax = "linguafranca"
