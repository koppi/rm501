#include <stdio.h>
#include <stdlib.h>
#include <curses.h>
#include <string.h>
#include <malloc.h>
#include <unistd.h>
#define MAX_INC 4000
#define MIN_INC 10
#define MAX_BASE 0
#define MIN_BASE -12000
#define MAX_SHOULDER 0
#define MIN_SHOULDER -5200
#define MAX_ELBOW 3600
#define MIN_ELBOW 0
#define MIN_PITCH 0
#define MAX_PITCH 2400
#define MIN_TURNHAND -2400
#define MAX_TURNHAND 2400
#define POS_SPACE 600
#define DEV_OUT "output.txt"

struct robot_pos {
    int base;
    int shoulder;
    int elbow;
    int pitch;
    int wrist;
    int grip;
    int saved;
    
}; 
struct robot_pos rpos ;
robot_pos *posdata; 
WINDOW *mainwin;

char * itoa(int i) {
    char * buf = NULL;
    buf=(char *)calloc(1024,1);
    sprintf(buf,"%d",i);
    return buf;
}

void display_pos() {
    move(2,0); printw("Base     :%5d  (y-x)",rpos.base);
    move(3,0); printw("Shoulder :%5d  (q-a)",rpos.shoulder);
    move(4,0); printw("Elbow    :%5d  (w-s)",rpos.elbow);
    move(5,0); printw("HandPitch:%5d  (e-d)",rpos.pitch);
    move(6,0); printw("HandTurn :%5d  (r-t)",rpos.wrist);
    move(7,0); printw("Hand     :  %s  (f-g)",rpos.grip ? "ZU ":"AUF");
}

void show_panel(int what, const char * value) {
    int i;
    switch (what) {
	case 0: display_pos();break;
	case 1: // current command
		move(22,0);	
		printw("cmd: %s                                    ",value);break;
	case 3: // progess for sending Data
		move(11,0);printw("Sende Daten...    ");
		move(11,18);printw("%s %%",value);break;
	case 4: // progess for sending program
		move(11,0);printw("Sende Programm... ");
		move(11,18);printw("%s %%",value);break;
	case 5: // data transfer complete
		for (i=2;i<=4;i++) { move(i,30);printw("                  "); }
		move(11,0); printw("Gesendet. Starte Programm...");break;
	case 6: // increment changed
		move(21,1); printw("inc: %s   ",value);break;
	case 2: // Position saved
	case 8: // Position loaded
		move(11,0); 
		printw("Position #%s %s       ",value,what == 8 ? "geladen.    ":"gespeichert.");
	case 7: // storage positions
		move(20,25); printw("|%4s",value);
		move (21,15);
		int curpos=atoi(value);
		for (i=curpos-10;i<=curpos+10;i++) {
		    if(i>0 && i<=POS_SPACE)
        		printw("%c",posdata[i].saved ? 'X':'.');
		    else printw(" ");
		} 
    }
    refresh();
}


void rsend(const char *cmd) {
    FILE *fp;
    show_panel(1,cmd);
    fp=fopen(DEV_OUT,"w");
    fputs(cmd ,fp);
    fputs("\n",fp);
    fclose(fp);
    usleep(10000);
}

void reset() {
    rpos.base=0;
    rpos.shoulder=0;
    rpos.elbow=0;
    rpos.pitch=0;
    rpos.wrist=0;
    rpos.grip=0;

    rsend("NT");
    rsend("SP9");
    rsend("GP7,5,3");
}

void rmove(int joint,int step) {
    char *buf=NULL;
    char *buf2=NULL;
    int i;
    buf=(char *)calloc(1024,1);
    buf2=(char *)calloc(1024,1);
    strcat (buf,"MI");
    for(i=1;i<=3;i++) {
	if(joint==i) {
	    sprintf(buf2,"%d",step);
	    strcat(buf,buf2);	    
	}
	else strcat(buf,"0");
	strcat(buf,",");
    }
    switch (joint) {
	case 4: sprintf(buf2,"%d,%d",step,-step);break;
	case 5: sprintf(buf2,"%d,%d",step,step);break;
	default: sprintf(buf2,"0,0");break;
    }
    strcat(buf,buf2);
    strcat(buf,",0");
    rsend(buf);
    free(buf);free(buf2);
}

void rmove_base(int step) {
    if ((rpos.base+step <= MAX_BASE) && (rpos.base+step >= MIN_BASE)) {
	rpos.base+=step;
	rmove(1,step);
    } else {
	if(rpos.base+step > MAX_BASE) { rmove(1,MAX_BASE-rpos.base); 
	                                 rpos.base=MAX_BASE; }
	if(rpos.base+step < MIN_BASE) { rmove(1,MIN_BASE-rpos.base);
					 rpos.base=MIN_BASE; }
    }

}

void rmove_shoulder(int step) {
    if ((rpos.shoulder+step <= MAX_SHOULDER) && (rpos.shoulder+step >= MIN_SHOULDER)) {
    	rpos.shoulder+=step;
	rmove(2,step);
    } else {
	if(rpos.shoulder+step > MAX_SHOULDER) { rmove(2,MAX_SHOULDER-rpos.shoulder); 
	                                 rpos.shoulder=MAX_SHOULDER; }
	if(rpos.shoulder+step < MIN_SHOULDER) { rmove(2,MIN_SHOULDER-rpos.shoulder);
					 rpos.shoulder=MIN_SHOULDER; }
    }
}

void rmove_elbow(int step) {
    if ((rpos.elbow+step <= MAX_ELBOW) && (rpos.elbow+step >= MIN_ELBOW)) {
	rpos.elbow+=step;
	rmove(3,step);
    } else {
	if(rpos.elbow+step > MAX_ELBOW) { rmove(3,MAX_ELBOW-rpos.elbow); 
	                                 rpos.elbow=MAX_ELBOW; }
	if(rpos.elbow+step < MIN_ELBOW) { rmove(3,MIN_ELBOW-rpos.elbow);
					 rpos.elbow=MIN_ELBOW; }
    }
}

void rmove_pitch(int step) {
    step/=2;
    if ((rpos.pitch+step <= MAX_PITCH) && (rpos.pitch+step >= MIN_PITCH)) {
	rpos.pitch+=step;
	rmove(4,step);
    } else {
	if(rpos.pitch+step > MAX_PITCH) { rmove(4,MAX_PITCH-rpos.pitch); 
	                                 rpos.pitch=MAX_PITCH; }
	if(rpos.pitch+step < MIN_PITCH) { rmove(4,MIN_PITCH-rpos.pitch);
					 rpos.pitch=MIN_PITCH; }
    }
}

void rmove_turnhand(int step) {
    if ((rpos.wrist+step <= MAX_TURNHAND) && (rpos.wrist+step >= MIN_TURNHAND)) {
	rpos.wrist+=step;
	rmove(5,step);
    } else {
	if(rpos.wrist+step > MAX_TURNHAND) { rmove(5,MAX_TURNHAND-rpos.wrist); 
	                                 rpos.wrist=MAX_TURNHAND; }
	if(rpos.wrist+step < MIN_TURNHAND) { rmove(5,MIN_TURNHAND-rpos.wrist);
					 rpos.wrist=MIN_TURNHAND; }
    }

}

void rmove_grip(int grip) {
    rpos.grip=grip;
    if (grip==0) rsend("GO");
    else rsend("GC");
}

void store_position(int posnum) {
    char *buf=NULL;
    buf=(char *)calloc(1024,1);
    posdata[posnum]=rpos;
    posdata[posnum].saved=1;
    show_panel(2,itoa(posnum));
    
    sprintf(buf,"HE%d",posnum);
    rsend(buf);
    free(buf);
}

void recall_position(int posnum) {
    char *buf=NULL;
    int t_base,
        t_shoulder,
	t_elbow,
	t_pitch,
	t_wrist;
    buf=(char *)calloc(1024,1);
    
    rsend( posdata[posnum].grip==0 ? "GO":"GC" ); 
    t_base    =posdata[posnum].base    -rpos.base;
    t_shoulder=posdata[posnum].shoulder-rpos.shoulder;
    t_elbow   =posdata[posnum].elbow   -rpos.elbow;
    t_pitch   =posdata[posnum].pitch   -rpos.pitch;
    t_wrist   =posdata[posnum].wrist   -rpos.wrist;
    
    rpos=posdata[posnum];

    sprintf(buf,"MI%d,%d,%d,%d,%d,0",t_base,t_shoulder,t_elbow,t_wrist+t_pitch,t_wrist-t_pitch);
    rsend(buf);
    free(buf);
    show_panel(8,itoa(posnum));
}

void run_program() {
    int i,lastpos=0,total_positions=0,sent_positions=0;
    char *buf=NULL;
    buf=(char *)calloc(255,1);
    rsend("NW");
    for(i=1;i<=POS_SPACE;i++) { if(posdata[i].saved==1) total_positions++; }
    for(i=1;i<=POS_SPACE;i++) {
	if(posdata[i].saved==1) {
	sprintf(buf,"PS %d,%d,%d,%d,%d,%d,0",i 
					    ,posdata[i].base
					    ,posdata[i].shoulder
					    ,posdata[i].elbow
					    ,posdata[i].wrist+posdata[i].pitch
					    ,posdata[i].wrist-posdata[i].pitch);
	    show_panel(3,itoa((sent_positions++*100)/total_positions));
	    if((i==1) || (posdata[i-1].grip != posdata[i].grip) ) {
		if(posdata[i].grip) rsend("GF1"); 
		else rsend("GF0");
	    }
	    rsend(buf);
	}
    }
    sent_positions=0;
    for(i=1;i<=POS_SPACE;i++) {
	if(posdata[i].saved==1) {
	    lastpos = i > lastpos ? i : lastpos;
	    sprintf(buf,"%d MO %d",i,i);
	    show_panel(4,itoa((sent_positions++*100)/total_positions));
	    rsend(buf);
	}
    }
    sprintf(buf,"%d ED",i+1);
    rsend(buf);
    rpos=posdata[lastpos];
    rsend("RN 1");
    free(buf);
    show_panel(5,"");
}

void save_posdata() {
    char *filename=NULL;
    FILE *fp;
    filename=(char *)calloc(1024,1);
    move(12,0); printw("Save Posdata to: ");refresh();
    echo();nocbreak();
    getstr(filename);
    noecho();cbreak();
    fp=fopen(filename,"w");
    move(12,0);printw("                                 ");
    refresh();
    fwrite((void *)posdata,sizeof(robot_pos),POS_SPACE,fp);
    fclose(fp);
    move(12,0);printw("                                 ");
    refresh();
    free(filename);
}

void load_posdata() {
    char *filename=NULL;
    FILE *fp;
    filename=(char *)calloc(1024,1);
    move(12,0); printw("Daten laden von: ");refresh();
    echo();nocbreak();
    getstr(filename);
    noecho();cbreak();
    fp=fopen(filename,"r");
    fread((void *)posdata,sizeof(robot_pos),POS_SPACE,fp);
    fclose(fp);
    move(12,0);printw("                                 ");
    refresh();
    free(filename);
}

void helpscreen() {
    char c;
	WINDOW * win;
	win=newwin(0,0,0,0);
	mvwprintw(win,0,0,"HILFE --- drücke 'q' zum verlassen");
	mvwprintw(win,2,0,"Der Roboter wird immer mit voller Geschwindigkeit bewegt");
	mvwprintw(win,3,0,"Der Anpressdruck der Hand ist ebenfalls fest vorgegeben");
	mvwprintw(win,5,0,"Die Steuerung erfolgt mittels Tasten. Welches Element des Arms");
	mvwprintw(win,6,0,"zu welchen Tasten gehört, kann man im Hauptbildschirm sehen");
	mvwprintw(win,8,0,"+ -                     : Schrittweite ändern");
	mvwprintw(win,9,0,"Leertaste               : Arm in Grundstellung");
	mvwprintw(win,10,0,"R                       : Hardware-Reset des Arms");
	mvwprintw(win,12,0,"--- Speichern der Positionen");
	mvwprintw(win,13,0,"Pfeiltasten links-rechts: Speicherposition auswählen");
	mvwprintw(win,14,0,", .                     : Speicherposition in 50er Schritten auswählen");
	mvwprintw(win,15,0,"Pfeiltaste  unten       : Position speichern");
	mvwprintw(win,16,0,"Pfeiltaste  oben        : Position laden");
	mvwprintw(win,17,0,"SHIFT-D                 : aktuelle Position auf NULL setzen");
	mvwprintw(win,18,0,"STRG-Pfeil-unten        : alle Positionen in Datei sichern");
	mvwprintw(win,19,0,"STRG-Pfeil-oben         : alle Positionen aus Datei laden");
	mvwprintw(win,21,0,"--- Roboterprogramm");
	mvwprintw(win,22,0,"STRG-G                  : alle Positionen als Programm an Roboter senden");
	mvwprintw(win,23,0,"SHIFT-G                 : Programm im Roboter starten");
	wrefresh(win);
	while (c!='q') { c=getch(); }
	wclear(win);
	wrefresh(win);
	delwin(win);
	redrawwin(mainwin);
}

void main_loop() {
    char c;
    int rinc=100,storage_pos=1;
    
    while (c!='Q') {
	display_pos();
        c=getch();
	if (c==27) c=getch();
	move(20,0);
	//printw("keycode: %d  ",c);
	switch (c)
	{
	    case  32:  reset();break;			/* SPACE */
	    case 104:					/* h */
	    case  72: helpscreen();break;		/* H */
	    case  82: rsend("RS");break;    		/* R */
	    case 121: 					/* y */
	    case 122: rmove_base(rinc);break;		/* z  */
	    case 120: rmove_base(-rinc);break;		/* x  */
	    case 113: rmove_shoulder(rinc);break;	/* q  */
	    case  97: rmove_shoulder(-rinc);break;	/* a  */
	    case 119: rmove_elbow(rinc);break;		/* w  */
	    case 115: rmove_elbow(-rinc);break;		/* s  */
	    case 101: rmove_pitch(rinc);break;		/* e  */
	    case 100: rmove_pitch(-rinc);break;		/* d  */
	    case 114: rmove_turnhand(rinc);break;	/* r  */
	    case 116: rmove_turnhand(-rinc);break;	/* t  */
	    case 102: rmove_grip(1);break;		/* f  */
	    case 103: rmove_grip(0);break;		/* g  */
	    case 45:  					/* -  */
		      rinc -= rinc/2;
		      if (rinc < MIN_INC) rinc=MIN_INC;
		      show_panel(6,itoa(rinc));
		      break;
	    case 43: 					/* +  */
		      rinc = rinc < 100 ? 100:rinc+100;
		      if (rinc > MAX_INC) rinc= MAX_INC;
		      show_panel(6,itoa(rinc));
		      break;

	    case 68: posdata[storage_pos].saved=0;     /* D */
		      show_panel(7,itoa(storage_pos));break;
	    case 44: storage_pos+=49;			/* , */
	    case  5: storage_pos++ ;			/* CRSR_left  */
		     if (storage_pos > POS_SPACE) storage_pos=POS_SPACE;
		     show_panel(7,itoa(storage_pos));
		     break;
	    case 46: storage_pos-=49;			/* . */
	    case  4: storage_pos-- ;			/* CRSR_right */
	             if (storage_pos < 1) storage_pos=1;
		     show_panel(7,itoa(storage_pos));
		     break;
	    case   2: store_position(storage_pos);break; /* CRSR_down  */
	    case   3: recall_position(storage_pos);break;/* CRSR_up  */
	    case  71: rsend("RN 1");break;		/* SHIFT_G  */
	    case   7: run_program();break;		/* CTRL_G */
	    case 65 : load_posdata();			/* CTRL_CRSR_UP" */
		      show_panel(7,itoa(storage_pos));break;
	    case 66 : save_posdata();break;		/* CTRL_CRSR_DOWN */
	}
    }
}
main () {
    int i;
    mainwin=initscr(); noecho(); cbreak();immedok(curscr,TRUE);
    keypad(stdscr, TRUE);
    posdata=(robot_pos *)calloc(POS_SPACE+1,sizeof(robot_pos));
    for(i=0;i<=POS_SPACE;i++) posdata[i].saved=0;
    printw("Robotersteuerung RM 501 V1.0\n");
    refresh();
    reset(); 
    main_loop();
    echo();
    endwin();
}

    
