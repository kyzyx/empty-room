plv_drawstyle -flipnorm 1
globalset noRedraw 1

set files ""
set fileid [open MANIFEST r]
set numchars [gets $fileid filename]
while {$numchars > 0} {
    lappend files $filename
    set numchars [gets $fileid filename]
}
close $fileid

foreach x $files {readfile $x}
globalset noRedraw 0
