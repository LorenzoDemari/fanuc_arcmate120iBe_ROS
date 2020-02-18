/PROG  ROS_MOVE
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "r2";
PROG_SIZE	= 518;
CREATE		= DATE 12-10-02  TIME 12:09:14;
MODIFIED	= DATE 13-04-28  TIME 18:20:26;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 25;
MEMORY_SIZE	= 890;
PROTECT		= READ_WRITE;
TCD:  STACK_SIZE	= 0,
      TASK_PRIORITY	= 50,
      TIME_SLICE	= 0,
      BUSY_LAMP_OFF	= 0,
      ABORT_REQUEST	= 0,
      PAUSE_REQUEST	= 0;
DEFAULT_GROUP	= 1,*,*,*,*;
CONTROL_CODE	= 00000000 00000000;
/MN
   1:   ;
   2:  !init: not rdy, no ack ;
   3:  R[1]=0    ;
   4:  R[2]=0    ;
   5:   ;
   6:  LBL[10] ;
   7:   ;
   8:  !we're ready for new point ;
   9:  R[1]=1    ;
  10:   ;
  11:  !wait for relay ;
  12:  WAIT R[2] ;
  13:   ;
  14:  !cache in temp preg ;
  15:  PR[2]=PR[1]    ;
  16:   ;
  17:  !first rdy low, then ack copy ;
  18:  R[1]=0    ;
  19:  R[2]=0    ;
  20:   ;
  21:  !move to point ;
  22:J PR[2] R[1]% CNT100    ;
  23:   ;
  24:  !done, repeat ;
  25:  JMP LBL[10] ;
/POS
/END
