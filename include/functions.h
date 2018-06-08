#include <kipr/botball.h>
#include <math.h>

#define LEFTWHEEL 0
#define RIGHTWHEEL 1
#define OPEN 1000
#define CLOSEBOTTOM 2000
#define CLOSETOP 300
#define RIGHTSIDE 1
#define LEFTSIDE 2

void go(double, int);
void distance(double, int);
void backwards(double, int);

void set_Claw_Position(int);
void set_ClawMachine(int, int);

void rotate_Right_Degrees(double, int);
void rotate_Left_Degrees(double, int);
void pull_out_left(double, int);
void pull_out_right(double,int);
void go_forward_and_right(double, int);
void go_forward_and_left(double, int);
void turn_left_until_tape(double, int);

void forwards_pid(int, double, double, int);
void backwards_pid_front_sensor(double, double, int);
void backwards_pid_back_sensor(int, double, double, int);

void getCubes();
void go_to_position1();
void go_to_position2();
void position1_placeCubes();
void position2_placeCubes();
void backToStart(double, double, double);

void block1();
void block2();
void block3();




void go(double centimeters, int speed)
{
double ticks = centimeters * 47;
    int start = 0;
    clear_motor_position_counter(LEFTWHEEL);
    
    while(get_motor_position_counter(LEFTWHEEL) < ticks)
    {
        while(start < speed)                   //slow start up until wheels are at speed
        {
            start = start + 50;
            mrp(LEFTWHEEL, start, 50);
            mrp(RIGHTWHEEL, start*0.96, 50);
            msleep(20);
        }
        
        while(start >= speed && gmpc(LEFTWHEEL) < ticks)		//goes forwards at set speed
        {
        	mrp(LEFTWHEEL, speed, 50);
        	mrp(RIGHTWHEEL, speed*0.96, 50);
        }
    }
}

void distance(double centimeters, int speed)
{
    double ticks = centimeters * 47;
    int start = 0;
    clear_motor_position_counter(LEFTWHEEL);
    
    while(get_motor_position_counter(LEFTWHEEL) < ticks)
    {
        while(start < speed)                   //slow start up until wheels are at speed
        {
            start = start + 50;
            mrp(LEFTWHEEL, start*1.04, 50);
            mrp(RIGHTWHEEL, start*0.91, 50);
            msleep(20);
        }
        
        while(start >= speed && gmpc(LEFTWHEEL) < ticks)		//goes forwards at set speed
        {
        	mrp(LEFTWHEEL, speed*1.04, 50);
        	mrp(RIGHTWHEEL, speed*0.91, 50);
        }
    }
}


void backwards(double centimeters, int speed)
{
    double ticks = centimeters * -47;
    int start = 0;
    clear_motor_position_counter(LEFTWHEEL);
    
    while(get_motor_position_counter(LEFTWHEEL) > ticks)
    {
        while(start < speed)                             //slow start up until wheels at speed
        {
            start = start + 50;
            mrp(LEFTWHEEL, start*1.055, -50);
            mrp(RIGHTWHEEL, start, -50);
            msleep(20);
        }
        
        while(start >= speed && gmpc(LEFTWHEEL) > ticks)        //goes backwards at set speed
        {
        	mrp(LEFTWHEEL, speed*1.05, -50);
        	mrp(RIGHTWHEEL, speed, -50);
        }
    }
}    



void set_Claw_Position(int pos)
{
    set_servo_position(0, pos);
    msleep(500);
}

void set_ClawMachine(int pos, int newpos)
{
    set_servo_position(1, pos);
    if(pos < newpos)
    {
        while(pos < newpos)
        {
        	pos = pos + 10;
            set_servo_position(1, pos);
            msleep(10);
        }
        set_servo_position(1, newpos);
        msleep(100);
    }
    
    if(pos > newpos)
    {
        while(pos > newpos)
        {
            pos = pos - 10;
            set_servo_position(1, pos);
            msleep(10);
        }
        set_servo_position(1, newpos);
        msleep(100);
    }
}


void rotate_Right_Degrees(double degrees, int speed) 
{
    double ticks = degrees * 20;
    int start = 0;
    clear_motor_position_counter(LEFTWHEEL);
    
    while(get_motor_position_counter(LEFTWHEEL) < ticks)		//slow start up until at speed
    {
        while(start < speed)
        {
            start = start + 50;
            mrp(LEFTWHEEL, start, 50);
            mrp(RIGHTWHEEL, start, -50);
            msleep(10);
        }
        
        while(start >= speed && gmpc(LEFTWHEEL) < ticks)        //rotates set degrees at set speed
        {
        	mrp(LEFTWHEEL, speed, 50);
            mrp(RIGHTWHEEL, speed, -50);
        }
    }
    
    bmd(RIGHTWHEEL);
    bmd(LEFTWHEEL);
}


void rotate_Left_Degrees(double degrees, int speed)
{
    double ticks = degrees * 20;
    int start = 0;
    clear_motor_position_counter(RIGHTWHEEL);
    
    while(get_motor_position_counter(RIGHTWHEEL) < ticks)		//slow start up until at set speed
    {
        while(start < speed)
        {
            start = start + 50;
            mrp(RIGHTWHEEL, start, 50);
            mrp(LEFTWHEEL, start, -50);
            msleep(20);
        }
        
        while(start >= speed && gmpc(RIGHTWHEEL) < ticks)		//rotate a set degrees at a set speed
        {
        	mrp(RIGHTWHEEL, speed, 50);
            mrp(LEFTWHEEL, speed, -50);
        }
    }
    
    bmd(RIGHTWHEEL);
    bmd(LEFTWHEEL);
}


void pull_out_left(double degrees, int speed)
{
    double ticks = degrees * -24;
    int start = 600;
    clear_motor_position_counter(RIGHTWHEEL);
    
    while(get_motor_position_counter(RIGHTWHEEL) > ticks)
    {
        while(start > speed && gmpc(RIGHTWHEEL) > ticks)		//drives backwards while slowing down left wheel
        {
            start = start - 100;
            mrp(LEFTWHEEL, start, -50);
            mrp(RIGHTWHEEL, 1000, -50);
            msleep(20);
            printf("start %i\n", start);
        }
        
        while(start <= speed && gmpc(RIGHTWHEEL) > ticks)         //turns rear left
        {
            mrp(LEFTWHEEL, speed, -50);
        	mrp(RIGHTWHEEL, 1000, -50);
            printf("L speed %i\n", speed);
            printf("%i\n", gmpc(RIGHTWHEEL));
        }
    }
    
    bmd(RIGHTWHEEL);
    bmd(LEFTWHEEL); 
}


void pull_out_right(double degrees, int speed)
{
    double ticks = degrees * -24;
    int start = 600;
    clear_motor_position_counter(LEFTWHEEL);
    
    while(get_motor_position_counter(LEFTWHEEL) > ticks)
    {
        
        while(start > speed && gmpc(LEFTWHEEL) > ticks)			//drives backwards while slowing down right wheel
        {
            start = start - 50;
            mrp(LEFTWHEEL, 1000, -50);
            mrp(RIGHTWHEEL, start, -50);
            msleep(50);
        }
        
        while(start >= speed && gmpc(LEFTWHEEL) > ticks)		//turns rear right
        {
            mrp(LEFTWHEEL, 1000, -50);
        	mrp(RIGHTWHEEL, speed, -50);
            printf("%i\n", gmpc(LEFTWHEEL));
        }
    }
    
    bmd(RIGHTWHEEL);
    bmd(LEFTWHEEL);
}


void go_forward_and_right(double degrees, int speed)
{
    double ticks = degrees * 24;
    int start = 600;
    clear_motor_position_counter(LEFTWHEEL);
    
    while(get_motor_position_counter(LEFTWHEEL) < ticks)
    {
        
        while(start > speed && gmpc(LEFTWHEEL) < ticks)			//drives backwards while slowing down right wheel
        {
            start = start - 50;
            mrp(LEFTWHEEL, 1000, 50);
            mrp(RIGHTWHEEL, start, 50);
            msleep(50);
        }
        
        while(start >= speed && gmpc(LEFTWHEEL) < ticks)		//turns rear right
        {
            mrp(LEFTWHEEL, 1000, 50);
        	mrp(RIGHTWHEEL, speed, 50);
            printf("%i\n", gmpc(LEFTWHEEL));
        }
    }
    
    bmd(RIGHTWHEEL);
    bmd(LEFTWHEEL);
}

void go_forward_and_left(double degrees, int speed)
{
    double ticks = degrees * 24;
    int start = 600;
    clear_motor_position_counter(RIGHTWHEEL);
    
    while(get_motor_position_counter(RIGHTWHEEL) < ticks)
    {
        while(start > speed && gmpc(RIGHTWHEEL) < ticks)		//drives backwards while slowing down left wheel
        {
            start = start - 150;
            mrp(LEFTWHEEL, start, 50);
            mrp(RIGHTWHEEL, 1000, 50);
            msleep(20);
            printf("start %i\n", start);
        }
        
        while(start <= speed && gmpc(RIGHTWHEEL) < ticks)         //turns rear left
        {
            mrp(LEFTWHEEL, speed, 50);
        	mrp(RIGHTWHEEL, 1000, 50);
            printf("L speed %i\n", speed);
            printf("%i\n", gmpc(RIGHTWHEEL));
        }
    }
    
    bmd(RIGHTWHEEL);
    bmd(LEFTWHEEL); 
}


void turn_left_until_tape(double perfect, int speed)
{
    double counter = 0;
    while(counter < perfect)
    {
        while(analog(0) > 3000 && counter < perfect)
        {
       		counter = counter + 1.12;
        	printf("%lf counter\n", counter);
            mrp(RIGHTWHEEL, speed, 5);
        	mrp(LEFTWHEEL, speed, -5);
            msleep(50);
        }
        while(analog(0) < 3000 && counter < perfect)
        {
        	mrp(RIGHTWHEEL, speed, 5);
        	mrp(LEFTWHEEL, speed, -5);
            counter=counter + 0.066;
            printf("%lf White counter\n", counter);
            
        }
    }
    
    bmd(RIGHTWHEEL);
    bmd(LEFTWHEEL);
}



void forwards_pid(int side, double perfect, double position, int speed)
{
    if(side == 1)
    {
    	double current = analog(0);
    	double error = 1.4*(perfect/current);
    	double powerup;
    	double powerdown;
    	cmpc(LEFTWHEEL);
    	
    	while(gmpc(LEFTWHEEL) < position)
    	{
    	    current = analog(0);
    	    error = 1.4*(perfect/current);
    	    powerup = (speed * error)*2;
    	    powerdown = (speed/error)*0.15;
            //printf("Error:%lf\n",error);
	
	        while(analog(0) == perfect && gmpc(LEFTWHEEL) < position)		//at perfect value drive straight forward
	        {
	        	current = analog(0);
	        	error = 1.4*(perfect/current);
	        	powerup = (speed * error)*2;
	        	powerdown = (speed/error)*0.15;
	            
	            mrp(LEFTWHEEL, speed, 5);
	            mrp(RIGHTWHEEL, speed, 5);
                //printf("Error:%lf\n",error);
	        }
	        
	        while(analog(0) < perfect && gmpc(LEFTWHEEL) < position)	//when sensing white power up right wheel to turn left
	        {
	        	current = analog(0);
	        	error = 1.4*(perfect/current);
	        	powerup = (speed * error)*2;
	        	powerdown = (speed/error)*0.15;
	            
	            mrp(LEFTWHEEL, powerdown, 5);
	            mrp(RIGHTWHEEL, powerup, 5);
                //printf("Error:%lf\n",error);
	            
	            printf("W %lf powerup R\n", powerup);
	            printf("W %lf powerdown L\n", powerdown);
	            //printf("W %lf error\n", error);
	        }
	        
	        while(analog(0) > perfect && gmpc(LEFTWHEEL) < position)	//when sensing black power up left wheel to turn right
	        {
	        	current = analog(0);
	        	error = 1.4*(perfect/current);
	        	powerdown = (speed * error)*0.15;
	        	powerup = (speed/error)*2;
	            
	            mrp(LEFTWHEEL, powerup, 5);
	            mrp(RIGHTWHEEL, powerdown, 5);
	            
	            printf("B %lf powerup L\n", powerup);
	            printf("B %lf powerdown R\n", powerdown);
	       		//printf("B %lf error\n", error);
	        }
	    }
    }
    
    if(side == 2)
    {
    	double current = analog(0);
    	double error = 1.4*(perfect/current);
    	double powerup;
    	double powerdown;
    	cmpc(LEFTWHEEL);
    	
    	while(gmpc(LEFTWHEEL) < position)
    	{
    	    current = analog(0);
    	    error = 1.4*(perfect/current);
    	    powerup = speed * error;
    	    powerdown = speed/error;
            printf("Error:%lf\n",error);
	
	        while(analog(0) == perfect && gmpc(LEFTWHEEL) < position)		//at perfect value drive straight forward
	        {
	        	current = analog(0);
	        	error = 1.4*(perfect/current);
	        	powerup = speed * error;
	        	powerdown = speed/error;
	            
	            mrp(LEFTWHEEL, speed, 6);
	            mrp(RIGHTWHEEL, speed, 6);
                printf("Error:%lf\n",error);
	        }
	        
	        while(analog(0) > perfect && gmpc(LEFTWHEEL) < position)	//when sensing white power up right wheel to turn left
	        {
	        	current = analog(0);
	        	error = 1.4*(perfect/current);
	        	powerdown = (speed * error) * 0.4;
	        	powerup = (speed/error) * 1.5;
	            
	            mrp(LEFTWHEEL, powerup, 6);
	            mrp(RIGHTWHEEL, powerdown, 6);
	            
	            //printf("W %lf powerup R\n", powerup);
	            //printf("W %lf powerdown L\n", powerdown);
	            printf("W %lf error\n", error);
	        }
	        
	        while(analog(0) < perfect && gmpc(LEFTWHEEL) < position)	//when sensing black power up left wheel to turn right
	        {
	        	current = analog(0);
	        	error = 1.2*(perfect/current);
	        	powerup = (speed * error)*0.2;
	        	powerdown = (speed/error)*2.3;
	            
	            mrp(LEFTWHEEL, powerdown, 6);
	            mrp(RIGHTWHEEL, powerup, 6);
	            
	            //printf("B %lf powerup L\n", powerup);
	            //printf("B %lf powerdown R\n", powerdown);
	       		printf("B %lf error\n", error);
	        }
	    }
    }
}


void backwards_pid_front_sensor(double perfect, double position, int speed)
{
    double ticks = position * -1;
    double current = analog(0);
    double error = 1.05*(perfect/current);
    double powerup;
    double powerdown;
    cmpc(LEFTWHEEL);
    
    while(gmpc(LEFTWHEEL) > ticks)
    {
        current = analog(0);
        error = 1.05*(perfect/current);
        powerup = speed * error;
        powerdown = speed/error;
        printf("%i\n", analog(1));

        while(analog(0) == perfect && gmpc(LEFTWHEEL) > ticks)		//at perfect value drive straight backwards
        {
        	current = analog(0);
        	error = 1.05*(perfect/current);
        	powerup = speed * error;
        	powerdown = speed/error;
            printf("%i\n", analog(1));
            
            mrp(LEFTWHEEL, speed, -5);
            mrp(RIGHTWHEEL, speed, -5);
        }
        
        while(analog(0) < perfect && gmpc(LEFTWHEEL) > ticks)	//when sensing white powerup right wheel to rear left
        {
        	current = analog(0);
        	error = 1.1*(perfect/current);
        	powerup = (speed * error) * 0.2;
        	powerdown = (speed/error) * 5;
            
            //printf("W %lf powerup\n", powerup);
        	//printf("W %lf powerdown\n", powerdown);
        	//printf("W %lf error\n", error);
            
            mrp(LEFTWHEEL, powerdown, -10);
            mrp(RIGHTWHEEL, powerup, -10);
        }
        
        while(analog(0) > perfect && gmpc(LEFTWHEEL) > ticks)	//when sensing black powerup right wheel to rear left
        {
        	current = analog(0);
        	error = 1.1*(perfect/current);
        	powerdown = speed * error;
        	powerup = speed/error;
            
            //printf("B %lf powerup\n", powerup);
        	//printf("B %lf powerdown\n", powerdown);
        	//printf("B %lf error\n", error);
            
            mrp(LEFTWHEEL, powerdown, -10);
            mrp(RIGHTWHEEL, powerup, -10);
        }
    }
}

void backwards_pid_back_sensor(int side, double perfect, double position, int speed)
{   
    if(side == 1)
    {
        double ticks = position * -1;
    	double current = analog(1);
    	double error = 1.1*(perfect/current);
    	double powerup;
    	double powerdown;
    	cmpc(LEFTWHEEL);
        
    	while(gmpc(LEFTWHEEL) > ticks)
    	{
    	    current = analog(1);
    	    error = 1.1*(perfect/current);
    	    powerup = speed * error;
    	    powerdown = speed/error;
	
	        while(analog(1) == perfect && gmpc(LEFTWHEEL) > ticks)		//at perfect value drive straight backwards
	        {
	        	current = analog(1);
	        	error = 1.1*(perfect/current);
	        	powerup = speed * error;
	        	powerdown = speed/error;
	            
	            mrp(LEFTWHEEL, speed, -7);
	            mrp(RIGHTWHEEL, speed, -7);
	        }
	        
	        while(analog(1) < perfect && gmpc(LEFTWHEEL) > ticks)	//when sensing white powerup right wheel to rear left
	        {
	        	current = analog(1);
	        	error = 1.1*(perfect/current);
	        	powerup = (speed * error);
	        	powerdown = (speed/error);
	            
	            //printf("W %lf powerup R\n", powerup);
	        	//printf("W %lf powerdown L\n", powerdown);
	        	//printf("W %lf error\n", error);
	            
	            mrp(LEFTWHEEL, powerdown, -7);
	            mrp(RIGHTWHEEL, powerup, -7);
	        }
	        	
	        while(analog(1) > perfect && gmpc(LEFTWHEEL) > ticks)	//when sensing black powerup left wheel to rear right
	        {
	        	current = analog(1);
	        	error = 1.25*(perfect/current);
	        	powerdown = (speed * error) * 0.2;
	        	powerup = (speed/error) * 5;
	            
	            //printf("B %lf powerup L\n", powerup);
	        	//printf("B %lf powerdown R\n", powerdown);
	        	//printf("B %lf error\n", error);
	            
	            mrp(LEFTWHEEL, powerup, -7);
	            mrp(RIGHTWHEEL, powerdown, -7);
	        }
        }
    }
    
    
    if(side == 2)
    {
        double ticks = position * -1;
    	double current = analog(1);
    	double error = 1.1*(perfect/current);
    	double powerup;
    	double powerdown;
    	cmpc(LEFTWHEEL);
        
	    while(gmpc(LEFTWHEEL) > ticks)
	    {
	        current = analog(1);
	        error = 1.1*(perfect/current);
	        powerup = speed * error;
	        powerdown = speed/error;
	
	        while(analog(1) == perfect && gmpc(LEFTWHEEL) > ticks)		//at perfect value drive straight backwards
	        {
	        	current = analog(1);
	        	error = 1.1*(perfect/current);
	        	powerup = speed * error;
	        	powerdown = speed/error;
	            
	            mrp(LEFTWHEEL, speed, -7);
	            mrp(RIGHTWHEEL, speed, -7);
	        }
	        
	        while(analog(1) > perfect && gmpc(LEFTWHEEL) > ticks)	//when sensing white powerup right wheel to rear left
	        {
	        	current = analog(1);
	        	error = 1.25*(perfect/current);
	        	powerup = (speed * error) * 5;
	        	powerdown = (speed/error) * 0.2;
	            
	            //printf("W %lf powerup R\n", powerup);
	        	//printf("W %lf powerdown L\n", powerdown);
	        	//printf("W %lf error\n", error);
	            
	            mrp(LEFTWHEEL, powerdown, -7);
	            mrp(RIGHTWHEEL, powerup, -7);
	        }
	        
	        while(analog(1) < perfect && gmpc(LEFTWHEEL) > ticks)	//when sensing black powerup left wheel to rear right
	        {
	        	current = analog(1);
	        	error = (perfect/current);
	        	powerdown = (speed/error) * 1.5;
	        	powerup = (speed * error) * 0.75;
	            
	            printf("W %lf powerup L\n", powerup);
	        	printf("W %lf powerdown R\n", powerdown);
	        	printf("W %lf error\n", error);
	            
	            mrp(LEFTWHEEL, powerup, -10);
	            mrp(RIGHTWHEEL, powerdown, -10);
	        }
	    }
    }
}


void getCubes()
{
    rotate_Left_Degrees(20.5, 800);						//turn left until sensing tape and line follow to cubes
    turn_left_until_tape(6, 1000);
    distance(1.5,1000);
    cmpc(LEFTWHEEL);
    
    while(analog(1) < 1000)
    {
        forwards_pid(RIGHTSIDE, 2400, 200, 1000);
    }
    forwards_pid(RIGHTSIDE, 2400, 600, 1200); 
    set_Claw_Position(CLOSEBOTTOM);						//get the cubes and pull out to left
    
    backwards(7, 1000);
}

void go_to_position1()
{
    pull_out_left(89, 200);
    while(analog(4) < 3000)								//line follow back until senses starting box corner
    {
        backwards_pid_back_sensor(RIGHTSIDE, 1200, 100, 1200);
    }
    
    printf("WE HAVE SENSED BLACK STARTING BOX\n");
    backwards_pid_back_sensor(RIGHTSIDE, 1200, 2000, 1500);        //line follow back to pom farm push pom farm pop
    printf("we pushed pom pom farm pom pom\n");
    
    while(analog(4) > 3000)
    {
    	forwards_pid(RIGHTSIDE, 1200, 10, 1200);					//line follow forward to rest position
        printf("%i\n", analog(4));
    }
    bmd(LEFTWHEEL);
    bmd(RIGHTWHEEL);
}

void go_to_position2()
{
    pull_out_right(93, 200);
    while(analog(3) < 3000)								//line follow back until senses starting box corner
    {
        backwards_pid_back_sensor(LEFTSIDE, 1200, 100, 1200);
    }
    
    printf("WE HAVE SENSED BLACK STARTING BOX\n");
    backwards_pid_back_sensor(LEFTSIDE, 1200, 1600, 1500);        //line follow back to pom farm push pom farm pop
    printf("we pushed pom pom farm pom pom\n");
    
    while(analog(3) > 3000)
    {
    	forwards_pid(LEFTSIDE, 2200, 10, 1200);					//line follow forward to rest positio
    }
    bmd(LEFTWHEEL);
    bmd(RIGHTWHEEL);
}

void position1_placeCubes()
{
    if(analog(0) < 3000)					//drive forward until sensing black
    {
        while(analog(0) < 3000)
    	{
    		mrp(LEFTWHEEL, 500, 50);
            mrp(RIGHTWHEEL, 500, 50);
    	}
    }
    msleep(100);
    rotate_Left_Degrees(4, 400);
    msleep(100);
    rotate_Right_Degrees(2, 400);
    msleep(100);
    set_Claw_Position(CLOSETOP);			//grab top cube and drive back
    
    backwards(30, 1000);
    set_Claw_Position(OPEN);				//drop cubes and push to middle
    msleep(500);
    distance(5, 800);
    
    set_Claw_Position(CLOSEBOTTOM);
    set_Claw_Position(OPEN);
    
    distance(15.5, 800);							//push cubes forward
    set_Claw_Position(CLOSEBOTTOM);			//drive back and pull out right
    backwards(1, 700);
    pull_out_left(90, 200);
    backwards(7, 700);
    
    rotate_Left_Degrees(43, 1000);			//turn and go forward into the zone and release cube
    distance(6, 700);
    while(analog(0) < 3000)
    {
    	mrp(LEFTWHEEL, 500, 50);
        mrp(RIGHTWHEEL, 500, 50);
    }
    distance(0.2, 700);
    set_Claw_Position(OPEN);
    msleep(100);
    backwards(5, 700);
    msleep(100);
    go(6, 700);
    rotate_Right_Degrees(2, 800);
    rotate_Left_Degrees(2, 800);
}

void position2_placeCubes()
{
    if(analog(0) < 3000)					//drive forward until sensing black
    {
        while(analog(0) < 3000)
    	{
    		mrp(LEFTWHEEL, 500, 50);
            mrp(RIGHTWHEEL, 500, 50);
    	}
    }
    msleep(100);
    rotate_Right_Degrees(4, 400);
    msleep(100);
    rotate_Left_Degrees(2, 400);
    msleep(100);
    set_Claw_Position(CLOSETOP);			//grab top cube and drive back
    
    backwards(30, 1000);
    set_Claw_Position(OPEN);				//drop cubes and push to middle
    msleep(500);
    distance(5, 800);
    
    set_Claw_Position(CLOSEBOTTOM);
    set_Claw_Position(OPEN);
    
    distance(15.5, 800);							//push cubes forward
    set_Claw_Position(CLOSEBOTTOM);			//drive back and pull out right
    backwards(1, 700);
    pull_out_right(100, 200);
    backwards(7, 700);
    
    rotate_Right_Degrees(56, 1000);			//turn and go forward into the zone and release cube
    distance(6, 700);
    while(analog(0) < 3000)
    {
    	mrp(LEFTWHEEL, 500, 50);
        mrp(RIGHTWHEEL, 500, 50);
    }
    distance(0.2, 700);
    set_Claw_Position(OPEN);
    msleep(100);
    backwards(5, 700);
    msleep(100);
    go(6, 700);
    rotate_Left_Degrees(2, 800);
    rotate_Right_Degrees(2, 800);
}

void backToStart(double degrees, double distance, double disback)
{
    while(analog(1) < 3000)					//drive backwards until sensing tape
    {
        mrp(LEFTWHEEL, 700, -50);
        mrp(RIGHTWHEEL, 700, -50);
    }
    mrp(LEFTWHEEL, 700, -400);
    mrp(RIGHTWHEEL, 700, -400);
    bmd(LEFTWHEEL);
    bmd(RIGHTWHEEL);
    
    rotate_Left_Degrees(degrees, 600);						//when on top of the starting box rotate left
    backwards(disback, 1000);
    backwards_pid_back_sensor(RIGHTSIDE, 1700, distance, 1000);	//and drive backwards into starting position
    rotate_Left_Degrees(43, 600);
    backwards(45, 900);
}



void block1()
{
    go(8.5, 800);
    go_forward_and_right(84, 200);
    
    position1_placeCubes();
    backToStart(43, 3050, 0);
}


void block2()
{
    forwards_pid(RIGHTSIDE, 2200, 3200, 1000);
    go_forward_and_right(87, 200);
    
    position1_placeCubes();
    backToStart(47, 2150, 80);					//6200
}


void block3()
{
    go(8.5, 800);
    go_forward_and_left(83, 200);
    distance(2, 800);
    
    position2_placeCubes();
    backToStart(42.3, 3850, 100);					//9000
}








