#    This is a part of LinuxCNC
#    Copyright 2006-2009 Jeff Epler <jepler@unpythonic.net>
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 2 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program; if not, write to the Free Software
#    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

# Load the Linuxcnc package, which defines variables for various useful paths
package require Linuxcnc

set ::program    $::argv0
set ::print_file [lindex $argv 1]
set ::debug_file [lindex $argv 0]

proc popup_msg {msg} {
  tk_messageBox \
    -title "$::program" \
    -type ok \
    -message "$msg"
}

proc create_report {} {
  set   info_file   /tmp/linuxcnc.info   ;# created by linuxcnc_info
  set   report_file /tmp/linuxcnc.report ;# created by this script
  exec  linuxcnc_info -s                 ;# -s: suppress opening by a viewer

  set   txt [.f2.t get 1.0 end]          ;# all text from text widget
  set   fd [open $report_file w]
  puts  $fd "Error report created by $::program:\n"
  puts  $fd $txt
  puts  $fd "-----------------------------------------------------------------------"
  puts  $fd "Info report created by linuxcnc_info:"
  close $fd
  exec  cat $info_file >> $report_file
  popup_msg "Report file created by linuxcnc_info:\n\n$report_file"
}

proc insert_file {w title f {p {}}} {
    if [catch {set fd [open $f r]
               set t [read $fd]
               close $fd
              } msg] {
       set t "$::program:\n   $msg" ;# info on failed open
       if {[string first dmesg $f] >= 0} {
          set sys ""
          catch {set sys [exec lsb_release -d]}
          set t "$t\n\ndmesg requires root privilege on this system\n$sys"
          set t "$t\nUse a terminal to issue the command: sudo dmesg"
       }
    }

    if {$p != {} && [regexp -all -linestop -lineanchor -indices $p $t match]} {
        set match [lindex $match 0]
        set t [string range $t $match end]
    }

    $w insert end "$title\n" title
    if {[string compare $t {}]} {
        $w insert end $t
    } else {
        $w insert end "(empty)\n"
    }
    $w insert end "\n"
}

#this procedure creates a file with error information included
proc create_error_file {p_file dbg_file} {
    set savedir ""
    while {$savedir eq ""} {
        set savedir [tk_chooseDirectory -initialdir $::env(HOME) -title "Choose a Directory to Save Error File..."]
    }
    
    set presdir $::env(PWD)
    set configdir [exec basename $presdir]
    cd $savedir
    file mkdir ccc_err
    file copy -force $p_file "ccc_err/print_file"
    file copy -force $dbg_file "ccc_err/debug_file"
    exec dmesg > ccc_err/dmesg_output.txt
    file copy -force $presdir "ccc_err/$configdir"
    exec zip -r ccc_err_[clock format [clock seconds] -format "%Y-%m-%d_%H%M%S"].zip ccc_err
    file delete -force ccc_err
    cd $presdir
}


wm ti . [msgcat::mc "LinuxCNC Errors"]
frame .f
label .f.b -bitmap error
<<<<<<< HEAD
label .f.l -justify l -wraplength 400 -text [msgcat::mc "LinuxCNC terminated with an error.  When reporting problems, please create a report file and include in your message."]
=======
label .f.l -justify l -wraplength 400 -text [msgcat::mc "LinuxCNC terminated with an error.  To report problems, please click Create Error File, select a folder (usually a flash drive) and click OK.  A file named ccc_err_<datecode>.zip will be created which you can send to us for analysis.  Click CLOSE to close window."]
>>>>>>> c60672b... Add oxyfuel.comp, thcenable.comp, improved error reporting. ~ccc1.2
pack .f.b -side left -padx 8 -pady 8
pack .f.l -side left
pack .f -side top -fill x -anchor w

frame .f2
text .f2.t -yscrollcommand [list .f2.s set] -height 24 -wrap word
scrollbar .f2.s -command [list .f2.t yview]
grid rowconfigure .f2 0 -weight 1
grid columnconfigure .f2 0 -weight 1
grid .f2.t -row 0 -column 0 -sticky nsew
grid .f2.s -row 0 -column 1 -sticky ns
.f2.t tag configure title -font {Helvetica 16 bold}
pack .f2 -fill both -expand 1

#test code*************************************
#.f2.t insert end "Hello World!\n"

#**********************************************

set fileID [open [lindex $argv 1] "r"]
set file_data [read $fileID]
close $fileID
if [ regexp "Could not retrieve mac address" $file_data ] {
    .f2.t insert end "\n***** Could not connect to the ethernet motion control hardware. *****\n\nPlease make sure the control box is powered on and connected before starting LinuxCNC.\n"
} else {
    insert_file .f2.t "Print file information:" [lindex $argv 1]
    insert_file .f2.t "Debug file information:" [lindex $argv 0]
}
.f2.t configure -state disabled

frame .f3
button .f3.b1 -text [msgcat::mc "Create Error File"] -underline 7 -command {create_error_file [lindex $argv 1] [lindex $argv 0]} -width 15 -padx 4 -pady 1
button .f3.b2 -text [msgcat::mc "Select All"] -underline 0 -command {.f2.t tag add sel 0.0 end; tk_textCopy .f2.t} -width 15 -padx 4 -pady 1
button .f3.b3 -text [msgcat::mc "Close"] -underline 0 -command {destroy .} -width 15 -padx 4 -pady 1
bind . <Key-e> {.f3.b1 flash; .f3.b1 invoke}
bind . <Key-s> {.f3.b2 flash; .f3.b2 invoke}
bind . <Key-c> {.f3.b3 flash; .f3.b3 invoke}
pack .f3.b1 -side left -anchor e -padx 4 -pady 4
pack .f3.b2 -side left -anchor e -padx 4 -pady 4
pack .f3.b3 -side left -anchor e -padx 4 -pady 4
pack .f3 -side top -anchor e
