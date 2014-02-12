# To be run with scanalyze -noui
#
# Reads MANIFEST and aligns each mesh to its neighbor
# circling around to include first and last as neighbors
# then does global registration

proc rename_gr {a b w} {
    set gr ".gr"
    set sep "^^^"
    set path "globalregs/"
    set one "1"
    if {$w == 0} {
        file delete $path$a$sep$b$gr
        file delete $path$b$sep$a$gr
        if {[file exists $path$a$sep$b$one$gr] == 1} {
            file rename $path$a$sep$b$one$gr $path$a$sep$b$gr
        } else {
            file rename $path$b$one$sep$a$gr $path$b$sep$a$gr
        }
    } else {
        if {[file exists $path$a$one$sep$b$gr] == 1} {
            file rename $path$a$one$sep$b$gr $path$a$sep$b$gr
        } else {
            file rename $path$b$sep$a$one$gr $path$b$sep$a$gr
        }
    }
}

set files ""
set fileid [open MANIFEST r]
set numchars [gets $fileid filename]
puts $numchars
while {$numchars > 0} {
    lappend files $filename
    set numchars [gets $fileid filename]
}
close $fileid

set numFiles [llength $files]
set ext ".xf"
set point ""
for {set i 0} {$i < $numFiles} {incr i} {
    if {$i == 0} {
        readfile [lindex $files $i]
    }
    set notMoving [file root [lindex $files $i]]
    if {$i == [expr $numFiles - 1]} {
        set moving $notMoving$point
        set notMoving [file root [lindex $files 0]]
    } else {
        set moving [file root [lindex $files [expr $i+1]]]
        if {$i != 0} {
            file copy -force -- $notMoving$ext $moving$ext
        }
        readfile [lindex $files [expr $i+1]]
    }
    set err [plv_icpregister 0.5 0 20 5 1 plane $moving $notMoving$point abs 3.0 1 400 0]
    if {$point == "1"} { rename_gr $notMoving $moving 1}
    puts [concat "Final ICP error: " $moving $notMoving$point $err]
    if {$err > 0.05 && $i != [expr $numFiles - 1]} {
        puts [concat "ICP failure: " $moving $notMoving$point "reverting to point ICP"]
        file copy -force -- $notMoving$ext $moving$ext
        readfile [lindex $files [expr $i+1]]
        set one "1"
        set err [plv_icpregister 0.6 0 80 5 1 point $moving$one $notMoving$point abs 3.0 1 400 0]
        if {$point == "1"} { rename_gr $notMoving $moving 1}
        rename_gr $notMoving$point $moving 0
        set moving $moving$one
        puts [concat "Final point ICP error: " $moving $notMoving$point $err]
        set point "1"
    } else {
        set point ""
    }
    saveScanMetaData $moving xform
}
#plv_globalreg init_import
#plv_globalreg register 0.01
fileWriteAllScanXforms


# plv_icpregister
#     sampling_rate \
#     do-normal-sampling? \
#     number-of-iterations \
#     cull-percent \
#     avoid-boundary? \
#     optimization-method {plane, point} \
#     mesh-to-move \
#     mesh-to-hold-still \
#     threshold-kind {abs, rel} \
#     threshold-value \
#     save-global-registration \
#     max-global-reg-pairs \
#     quality-of-global-reg [I don't get this]
# 
# plv_icpregister
#    0.1
#    0
#    6
#    20
#    1
#    plane
#    move-mesh
#    stay-mesh
#    abs
#    5
#    1
#    200
#    0
# 
