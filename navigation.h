//Library for use with Robob Ross for navigation

int verify = 0; //counter to verify sensor reading
float Kp_nav;
float Kd_nav;
float Ki_nav;
float PID_vel;
long int encoder_dist[4];
float wall_dist;

void find_wall(void); //initial move command to find wall at beginning of program
void parallel_rough_adjust(void); //rough adjust to make base parallel with wall
void parallel_fine_adjust(void); //fine adjust to make base parallel with wall
void parallel_pid_adjust(void);
void right_PID_move_parallel(void);
void maintain_distance(void); //used to maintain a fixed distance from the wall
void maintain_distance_pid(void);
void right_move_parallel(void); //used to make the base move to the right parallel with the wall
void turn_corner(void);

void find_wall(void)
{
  RC = 0.065;
  wvel = 0.3;
  verify = 0;
  while (verify < 3)
  {
    while (sensor[0] < 300 && sensor[1] < 300)
    {
      set_move_command(1);
    }
    set_move_command(0);
    delay(100);
    verify++;
  }
}

void parallel_rough_adjust(void)
{
  RC = 0.01;
  wvel = 0.5;
  while (abs(sensor[1] - sensor[0]) > 50)
  {
    if (sensor[1] - sensor[0] > 50)
    {
      set_move_command(6);
    }
    else if (sensor[1] - sensor[0] < -50)
    {
      set_move_command(5);
    }
    else
      set_move_command(0);
  }
}

void parallel_pid_adjust(void)
{
  wvel = 0.1;
  RC = 0.10;
  Kp_nav = 0.020;
  Kd_nav = 0.0005;
  verify = 0;
  //delay(100);
  while (abs(theta) > 10)
  {
    while (abs(theta) > 10)
    {
      PID_vel = Kp_nav * theta + Kd_nav * ((theta - theta_prev) / T);
      if (PID_vel > 0.1)
      {
        wvel = 0.1;
      }
      else if (PID_vel < -0.1)
      {
        wvel = -0.1;
      }
      else
      {
        wvel = PID_vel;
      }
      set_move_command(6);
    }
    set_move_command(0);
    delay(100);
    verify++;
  }
}

void right_PID_move_parallel(void)
{
  wvel = 0.25;
  RC = 0.15;
  Kp_nav = 0.0008;
  Kd_nav = 0.0005;
  verify = 0;
  delay(100);
  while (verify < 2)
  {
    while (sensor[2] < 400)
    {
      if (abs(theta) > 20)
      {
        PID_vel = Kp_nav * theta + Kd_nav * ((theta - theta_prev) / T);
        wvel = 0.25;
        //PID_vel = Kp_nav*theta;
        vel_offset[0] = -PID_vel;
        vel_offset[1] = -PID_vel;
        vel_offset[2] = PID_vel;
        vel_offset[3] = PID_vel;
      }
      //    else if (fsensor > 340 || fsensor < 280)
      //    {
      //      PID_vel = Kp_nav*(fsensor - 300) + Kd_nav*(((fsensor-300) - (fsensor_prev-300))/T);
      //      wvel = 0.25;
      //      //PID_vel = Kp_nav*(fsensor - 300);
      //      vel_offset[0] = PID_vel;
      //      vel_offset[1] = -PID_vel;
      //      vel_offset[2] = PID_vel;
      //      vel_offset[3] = -PID_vel;
      //    }
      else
      {
        wvel = 0.25;
        vel_offset[0] = 0;
        vel_offset[1] = 0;
        vel_offset[2] = 0;
        vel_offset[3] = 0;
      }
      set_move_command(4);
    }
    set_move_command(0);
    delay(100);
    verify++;
  }
}

void parallel_fine_adjust(void)
{
  wvel = 0.1;
  RC = 0.15;
  delay(100);
  while (abs(sensor[1] - sensor[0]) > 10)
  {
    while (abs(sensor[1] - sensor[0]) > 10)
    {
      if (sensor[1] - sensor[0] > 10)
      {
        set_move_command(6);
      }
      else if (sensor[1] - sensor[0] < -10)
      {
        set_move_command(5);
      }
      else
        set_move_command(0);
    }
    delay(100); //delay to ensure base did not overshoot
  }
}

void maintain_distance(void)
{
  wvel = 0.1;
  RC = 0.15;
  delay(100);
  while (fsensor > 320 || fsensor < 280 || abs(theta) > 10)
  {
    while (fsensor > 320 || fsensor < 280)
    {
      if (fsensor > 320)
      {
        set_move_command(2);
      }
      else if (fsensor < 280)
      {
        set_move_command(1);
      }
      else
      {
        set_move_command(0);
      }
      if (abs(theta) > 5)
      {
        parallel_pid_adjust();
      }
    }
    set_move_command(0);
    delay(150); //delay to ensure base did not overshoot
  }
}

void maintain_distance_pid(void)
{
  wvel = 0.1;
  Kp_nav = 0.020;
  Kd_nav = 0.0005;
  RC = 0.15;
  //delay(100);
  while (fsensor > 320 || fsensor < 280 || abs(theta) > 10)
  {
    while (fsensor > 320 || fsensor < 280 || abs(theta) > 10)
    {
      PID_vel = Kp_nav * (fsensor - 300) + Kd_nav * (((fsensor - 300) - (fsensor_prev - 300)) / T);
      if (PID_vel > 0.1)
      {
        wvel = 0.1;
      }
      else if (PID_vel < -0.1)
      {
        wvel = -0.1;
      }
      else
      {
        wvel = PID_vel;
      }
      set_move_command(2);
      if (abs(theta) > 5)
      {
        parallel_pid_adjust();
      }
    }
    set_move_command(0);
    //delay(150); //delay to ensure base did not overshoot
  }
}

void right_move_parallel(void)
{
  while (sensor[2] < 400)
  {
    while (sensor[2] < 400)
    {
      wvel = 0.25;
      RC = 0.10;
      if (fsensor > 350 || fsensor < 250 || abs(theta) > 30)
      {
        maintain_distance_pid(); //correct position if base is too close or far from the wall
      }
      else if (sensor[2] < 400)
      {
        wvel = 0.25;
        RC = 0.1;
        set_move_command(4);
      }
      else
      {
        set_move_command(0);
      }
    }
    set_move_command(0);
    delay(100); //delay to ensure base did not overshoot
  }
  set_move_command(0);
}

void turn_corner(void)
{
  wvel = 0.25;
  RC = 0.1;
  delay(100);
  set_move_command(3);
  wait_until_move_done();
  delay(100);
  set_move_command(6);
  wait_until_move_done();
  delay(100);
  maintain_distance_pid();
  while (sensor[3] > 320 || sensor[3] < 280)
  {
    wvel = 0.1;
    while (sensor[3] > 320 || sensor[3] < 280)
    {
      if (sensor[3] > 320)
      {
        set_move_command(4);
      }
      else if (sensor[3] < 280)
      {
        set_move_command(3);
      }
    }
    set_move_command(0);
    delay(150);
  }
}

void right_move_parallel_measure(void)
{
  encoder_dist[0] = encoder0_val;
  encoder_dist[1] = encoder2_val;
  wall_dist = 0.0;
  while (sensor[2] < 400)
  {
    while (sensor[2] < 400)
    {
      wvel = 0.25;
      RC = 0.10;
      if (fsensor > 350 || fsensor < 250 || abs(theta) > 30)
      {
        maintain_distance_pid(); //correct position if base is too close or far from the wall
      }
      else if (sensor[2] < 400)
      {
        wvel = 0.25;
        RC = 0.1;
        set_move_command(4);
      }
      else
      {
        set_move_command(0);
      }
    }
    set_move_command(0);
    //delay(100); //delay to ensure base did not overshoot
  }
  encoder_dist[2] = encoder0_val;
  encoder_dist[3] = encoder2_val;
  wall_dist = ((abs(encoder_dist[0] - encoder_dist[2])) + (abs(encoder_dist[1] - encoder_dist[3])))/2.0;
  wall_dist = wall_dist * 2.763e-5;
  set_move_command(0);
}
