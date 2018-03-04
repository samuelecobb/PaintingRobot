

/*Contains movement functions left, right, forward*/

volatile long int global_ticks = 0;
volatile int      move_status  = 0;
volatile int      move_command = 0;  /* 0 - stop 1 - left turn; 2- forward ; 3 - right turn ; 4 u turn  */

/*function prototypes*/

void    wait_until_move_done(void);
void    left_turn(void);
void    right_turn(void);
void    move_left(void);
void    move_right(void);
void    forward(void);
void    backward(void);
void    set_move_command(int command);

void    move_mr(int mc);
void    stop_mr(void);


/*vel correction is intended to compensate misaplignment of the wheels*/

void vel_correction(void)
{
  des_wvel[0] = 1.0 * des_wvel[0];
  des_wvel[1] = 1.0 * des_wvel[1];
  des_wvel[2] = 1.0 * des_wvel[2];
  des_wvel[3] = 1.0 * des_wvel[3];
}


/* The turn_time and f_time are specific to a robot.
   f_time     - move one grid
   turn_time  - turn 90 deg.
   Change/calibrate them for your robot */

int      turn_time    = 1000;
int      f_time       = 320;

void move_left(void)
{


  if (global_ticks < f_time)
  {
    des_wvel[0] = -wvel;
    des_wvel[1] =  wvel;
    des_wvel[2] = -wvel;
    des_wvel[3] =  wvel;
  }

  else
  {
    /* stop  */
    des_wvel[0]  =   0.;
    des_wvel[1]  =   0.;
    des_wvel[2]  =   0.;
    des_wvel[3]  =   0.;
    /*done moving*/
    move_status  =  0;

  }

  vel_correction();

}

void move_right(void)
{


  if (global_ticks < f_time)
  {
    des_wvel[0] =  wvel + vel_offset[0];
    des_wvel[1] = -wvel + vel_offset[1];
    des_wvel[2] =  wvel + vel_offset[2];
    des_wvel[3] = -wvel + vel_offset[3];
  }

  else
  {
    /* stop  */
    des_wvel[0]  =   0.;
    des_wvel[1]  =   0.;
    des_wvel[2]  =   0.;
    des_wvel[3]  =   0.;
    /*done moving*/
    move_status  =  0;

  }

  vel_correction();

}

void left_turn(void)
{


  if (global_ticks < turn_time)
  {
    des_wvel[0] = -wvel;
    des_wvel[1] = -wvel;
    des_wvel[2] =  wvel;
    des_wvel[3] =  wvel;
  }

  else
  {
    /* stop  */
    des_wvel[0]  =   0.;
    des_wvel[1]  =   0.;
    des_wvel[2]  =   0.;
    des_wvel[3]  =   0.;
    /*done moving*/
    move_status  =  0;

  }

  vel_correction();

}


void right_turn(void)
{


  if (global_ticks < turn_time)
  {
    des_wvel[0] = wvel;
    des_wvel[1] = wvel;
    des_wvel[2] = -wvel;
    des_wvel[3] = -wvel;
  }

  else
  {
    /* stop  */
    des_wvel[0]  =   0.;
    des_wvel[1]  =   0.;
    des_wvel[2]  =   0.;
    des_wvel[3]  =   0.;
    /*done moving*/
    move_status  =  0;

  }

  vel_correction();

}


void forward(void)
{

  if (global_ticks < f_time)
  {
    des_wvel[0] =   wvel;
    des_wvel[1] =   wvel;
    des_wvel[2] =   wvel;
    des_wvel[3] =   wvel;
  }

  else
  {
    /* stop  */
    des_wvel[0]  =   0.;
    des_wvel[1]  =   0.;
    des_wvel[2]  =   0.;
    des_wvel[3]  =   0.;
    /*done moving*/
    move_status  =  0;

  }

  vel_correction();

}

void backward(void)
{

  if (global_ticks < f_time)
  {
    des_wvel[0] =   -wvel;
    des_wvel[1] =   -wvel;
    des_wvel[2] =   -wvel;
    des_wvel[3] =   -wvel;
  }

  else
  {
    /* stop  */
    des_wvel[0]  =   0.;
    des_wvel[1]  =   0.;
    des_wvel[2]  =   0.;
    des_wvel[3]  =   0.;
    /*done moving*/
    move_status  =  0;

  }

  vel_correction();

}

void stop_mr(void)
{

  if (global_ticks < turn_time)
  {
    des_wvel[0] =   0.;
    des_wvel[1] =   0.;
    des_wvel[2] =   0.;
    des_wvel[3] =   0.;
  }

  else
  {
    move_status  =  0;
  }

}



void move_mr(int mc)
{


  switch (mc)
  {
    case 0:
      stop_mr();
      break;

    case 1:
      forward();
      break;

    case 2:
      backward();
      break;

    case 3:
      move_left();
      break;

    case 4:
      move_right();
      break;

    case 5:
      left_turn();
      break;

    case 6:
      right_turn();
      break;

    default:
      stop_mr();

  }

}

void set_move_command(int command)
{
  move_command = command;
  global_ticks = 0;
  move_status  = 1;
}



void wait_until_move_done(void)
{

  while (move_status != 0)
  {
    //Serial.print("");
  }

}












