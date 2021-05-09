#ifndef AUTON
#define AUTON

double get_stop_watch();
void start_stop_watch();
void debug_autonomous();
void run_homerow(void);
void run_skills(void);
void autonomous();

#endif


// old robot skills
/*

    void run_skills_start()
    {
        drivetrain.reset_odom();

        // =========== SCORE IN TOWER ONE ===========
        scorer.reset_balls_counted();
        scorer.set_flywheel(127);
        scorer.wait_until_number_of_upper_balls_counted(1);
        pros::delay(200);
        // =========== TURN OFF FLYWHEEL AND TURN ON INTAKES/INDEXERS ===========
        // =========== BACK OUT OF TOWER ONE ===========
        // =========== WAYPOINT TO BALL B ===========
        drivetrain.drive_to_point(
            7.88, -16.73, 196.72, 2, 3, []() {scorer.set_indexers(-127); scorer.set_intakes(127); scorer.set_flywheel(0); }, 15);

        // =========== MAKE SURE INTAKES HAVE DEPLOYED AND TURN INDEXERS POSITIVELY ===========
        scorer.set_indexers(0);
        scorer.set_intakes(0);
        pros::delay(200);
        scorer.set_indexers(127);
        scorer.set_intakes(127);

        // =========== COLLECT TO BALL B ===========
        drivetrain.drive_to_point(7.87, -14.05, 198.68, 2, 3, NULL, 0, 1000, true);

        scorer.set_intakes(0); //DELETE LATER

        // =========== DRIVE TO TOWER TWO ==========
        // drivetrain.drive_to_point(-8.71, -36.73, 227.34, 2, 3,x NULL, 0, 2000);
        // //changed to 2000 drivetrain.stop_drive_motors();
        drivetrain.drive_to_point(-7.73, -41.05, 242.60, 2, 3, NULL, 0, 2500);
        drivetrain.drive_to_point(-9.51, -42.77, 243.58, 2, 3, NULL, 0, 750);

        // =========== SCORE IN TOWER TWO ===========
        scorer.reset_balls_counted();
        scorer.score_n_balls(1, 1250, false, 63);
        // scorer.collect_n_balls(2);

        // =========== RESET GLOBAL POSITION ===========
        drivetrain.center_on_tower_with_bumper(245.00, false, 1000);
        drivetrain.reset_odom();
        // drivetrain.stop_drive_motors();
    }

    void run_field_sides(bool is_second_call)
    {
        // scorer.collect_n
        // scorer.collect_n_balls(2);

        // =========== FINISH SCORING AND COLLECTING IN TOWER TWO/SIX ===========
        scorer.wait_until_number_of_upper_balls_counted(1);
        // scorer.wait_until_number_of_lower_balls_counted(2);

        // =========== RESET BALLS COLLECTED ===========
        scorer.reset_balls_to_score();

        // // =========== DISPENSE TWO BALLS ===========
        // scorer.reset_balls_counted();
        // scorer.dispense_n_balls(2, true);

        // =========== WAYPOINT TO BALL C/H ===========
        // drivetrain.drive_to_point(
        //     -0.08, -24.61, 314.92, 1, 3);

        // =========== TURN ON INDEXERS AND INTAKES POSITIVELY ===========
        scorer.set_flywheel(0);

        // =========== WAYPOINT TO BALL C/H ===========
        drivetrain.drive_to_point(0.89, -8.52, 359.71, 1, 1, NULL, 0, 500);

            scorer.set_intakes(126);
        scorer.set_indexers(64);

        // =========== COLLECT BALL C/H ===========
        drivetrain.drive_to_point(-8.23, -8.49, 286.33, 2, 3, NULL, 0, 1500, true);

        // =========== WAYPOINT TO BALL D/ ===========
        // (figure out later)
        drivetrain.drive_to_point(-2.94, -48.35, 225.09, 2, 3, NULL, 0, 4500, false);

        // =========== COLLECT BALL D/
        // (figure out later) ===========
        drivetrain.drive_to_point(-10.69, -52.85, 229.10, 2, 3, NULL, 0, 500, true); //

        // =========== COLLECT BALL E/
        // (figure out later)
        drivetrain.drive_to_point(-26.18, -56.77, 316.27, 2, 3, NULL, 0, 1000, true);
        // AND DRIVE TO TOWER THREE/SEVEN ===========
        
        drivetrain.drive_to_point(-34.48, -42.41, 315.58, 2, 3, NULL, 0, 1500, false);
        drivetrain.stop_drive_motors();
        return;

        // drivetrain.drive_to_point(-35.81, -43.22, 316.27, 2, 3, NULL, 7, 2000);

        // // scorer.set_indexers(0);
        // // scorer.set_intakes(3);
        // if (is_second_call)
        // {
        // //   drivetrain.drive_to_point(-20.10, -32.76, 220.98, 2, 3, NULL, 0, 4200, true);
        //     // drivetrain.drive_to_point(-25.24, -38.47, 219.12, 2, 3, NULL, 0, 4200, true);
        //     drivetrain.drive_to_point(-23.28, -39.49, 220.88, 2, 3, [](){scorer.collect_n_balls(1, 6);}, 6, 4200);
        
        // }
        // else
        // {
        // drivetrain.drive_to_point(-25.39, -37.72, 222.16, 2, 3, [](){scorer.collect_n_balls(1, 6);}, 6, 4200);
        // // drivetrain.drive_to_point(
        // //     -26.72, -38.07, 223.13, 2, 3, []() {  }, 3, 750);
        // }


        // // =========== COLLECT BALL D/I ===========
        // // drivetrain.drive_to_point(-26.5, -44.72, 217.36, 2, 3, NULL, 0, 1000, true);

        // // =========== TURN ON FLYWHEEL AND INDEXERS slightly NEGATIVELY ===========
        // // scorer.set_flywheel(-31);
        // // scorer.set_indexers(-4);

        // // =========== WAYPOINT TO TOWER THREE/SEVEN AND TURN OFF FLYWHEEL AND INDEXERS ===========
        // // drivetrain.drive_to_point(
        // //     -28.85, -48.58, 223.67, 1, 3, []() {  }, 3, 500);

        // // scorer.set_indexers(60);
        // scorer.set_intakes(0); // DELETE LATER
        scorer.set_indexers(0);
        // scorer.set_indexers(127);

        // =========== DRIVE TO TOWER THREE/SEVEN ===========
        // drivetrain.drive_to_point(-28.66, -47.31, 312.06, 2, 3, NULL, 0, 1000);
        // drivetrain.drive_to_point(-35.81, -43.22, 316.27, 2, 3, NULL, 7, 1500);
        
        // =========== SCORE AND COLLECT IN TOWER THREE/SEVEN ===========
        scorer.reset_balls_counted();
        scorer.set_indexers(127);
        scorer.set_flywheel(127);
        scorer.score_n_balls(2, 1350, true);
        

        // scorer.collect_n_balls(1);

        // =========== RESET GLOBAL POSITION ===========

        drivetrain.center_on_tower_with_bumper(316.27, false, 1000);
        drivetrain.stop_drive_motors();
        drivetrain.reset_odom();
            scorer.set_indexers(127);
        scorer.set_flywheel(127);
        


        // =========== FINISH SCORING AND COLLECTING IN TOWER THREE/SEVEN ===========
        // scorer.wait_until_number_of_lower_balls_counted(1);
        scorer.wait_until_number_of_upper_balls_counted(2);
        if (is_second_call)
        {
                scorer.set_indexers(127);
                scorer.set_flywheel(127);
                pros::delay(600);
        
        }


        // =========== RESET BALLS COLLECTED ===========
        scorer.reset_balls_to_score();

        // =========== DISPENSE BALLS FROM TOWER THREE/SEVEN ===========
        // scorer.reset_balls_counted();
        // scorer.dispense_n_balls(1);
        // scorer.wait_until_number_of_balls_dispensed(1);

        // =========== BACK OUT OF TOWER THREE/SEVEN ===========
        drivetrain.drive_to_point(-10.31, -8.15, 322.04, 1, 1, NULL, 0, 1000);

        // =========== WAYPOINT TO BALL E/J ===========
        // =========== COLLECT BALL E/J ===========
        drivetrain.drive_to_point(-31.53, 4.66, 345.23, 2, 3, [](){scorer.set_intakes(127);scorer.set_indexers(127); scorer.set_flywheel(0);}, 30, 1550);

        // =========== COLLECT BALL E/J ===========
        // drivetrain.drive_to_point(-52.32, 0.51, 316.07, 2, 3, []() {scorer.set_intakes(127);scorer.set_indexers(127); scorer.set_flywheel(0); }, 30, 1000, true);

        // =========== TURN OFF INTAKES ===========
        // scorer.set_intakes(0);

        // =========== WAYPOINT TO TOWER FOUR/EIGHT ===========
        // drivetrain.drive_to_point(-44.81, 6.34, 5.09, 2, 1, NULL, 0, 750);// changed to 750 from 1000

        // =========== DRIVE TO TOWER FOUR/EIGHT ===========
        is_second_call ? drivetrain.drive_to_point(-55.13, 6.30, 321.65, 2, 3, [](){scorer.set_intakes(0);}, 3, 1750) : drivetrain.drive_to_point(-53.66, 2.33, 315.68, 2, 3, [](){scorer.set_intakes(0);}, 3, 1750);
        // drivetrain.stop_drive_motors();

        // =========== SCORE AND COLLECT IN TOWER FOUR/EIGHT ===========
        scorer.reset_balls_counted();

        scorer.set_intakes(0); // DELETE LATER

        is_second_call ? scorer.score_n_balls(2, 1250, false, 63) : scorer.score_n_balls(1, 1250, false, 63);

        
        // =========== RESET GLOBAL POSITION ===========
        drivetrain.center_on_tower_with_bumper(317, false, 1000);
        // pros::delay(200);
        drivetrain.reset_odom();



        // =========== FINISH SCORING AND COLLECTING IN TOWER FOUR/EIGHT ===========
        // scorer.wait_until_number_of_lower_balls_counted(1);
        scorer.wait_until_number_of_upper_balls_counted(1);


        // =========== RESET BALLS COLLECTED ===========
        scorer.reset_balls_to_score();
    }

    void run_blue_front() {
        // =========== BACK OUT OF TOWER FOUR ===========
        drivetrain.drive_to_point(9.78, -21.81, 283.01, 1, 1, NULL, 0, 1500);

        // =========== TURN ON INTAKES AND INDEXERS ===========
        scorer.set_intakes(127);
        scorer.set_indexers(127);

        // =========== COLLECT TO BALL F ===========
        drivetrain.drive_to_point(-4.34, -16.79, 274.69, 2, 2, NULL, 0, 1150);

        scorer.set_intakes(0); // DELETE LATER

        // =========== DRIVE TO TOWER FIVE ===========
        drivetrain.drive_to_point(-32.24, -46.29, 316.07, 2, 3, NULL, 0, 2500, true);

        drivetrain.drive_to_point(-35.04, -43.25, 318.81, 2, 3, NULL, 0, 500, true);

        // =========== WAYPOINT TO TOWER FIVE =========== //
        // drivetrain.drive_to_point(-27.59, -47.02, 313.64, 2, 3);

        
                            
        // =========== DRIVE TO TOWER FIVE =========== //
        // drivetrain.drive_to_point(-33.84, -46.13, 317.35, 2, 3, NULL, 0, 2500);

        // =========== SCORE AND COLLECT IN TOWER FIVE ===========
        scorer.reset_balls_counted();
        // scorer.collect_n_balls(1);
        scorer.score_n_balls(1);
        // =========== RESET GLOBAL POSITION ===========
        drivetrain.center_on_tower_with_bumper(320.67, false, 1000);
        
        //   pros::delay(200);
        drivetrain.reset_odom();
        //   drivetrain.stop_drive_motors();

        // =========== FINISH SCORING AND COLLECTING IN TOWER FIVE ===========
        //   scorer.wait_until_number_of_lower_balls_counted(1);
        scorer.wait_until_number_of_upper_balls_counted(1);

        // =========== RESET BALLS COLLECTED ===========
        scorer.reset_balls_to_score();

        // =========== BACK OUT OF TOWER FIVE ===========
        drivetrain.drive_to_point(-18.04, -14.75, 302.87, 2, 1, NULL, 0, 1300);
        // drivetrain.drive_to_point(-15.80, -5.96, 272.44, 2, 3, NULL, 0, 1000, true);

        // =========== TURN ON INDEXERS AND INTAKES ===========
        scorer.set_indexers(127);
        scorer.set_intakes(127);
        scorer.set_flywheel(0);

        // =========== DRIVE TO TOWER SIX ===========
        drivetrain.drive_to_point(-51.57, 2.10, 317.64, 2, 1, []() { scorer.set_intakes(0); }, 10, 1750); 

        // =========== SCORE AND COLLECT IN TOWER SIX ===========
        scorer.reset_balls_counted();

        

            //  scorer.set_flywheel(127);

        scorer.score_n_balls(1, 1300);

        // scorer.collect_n_balls(2);

        // =========== RESET GLOBAL POSITION ===========
        drivetrain.center_on_tower_with_bumper(314.61, false, 1000);
        drivetrain.reset_odom();

        scorer.set_indexers(127);
        scorer.set_flywheel(127);
        pros::delay(200);
        // drivetrain.stop_drive_motors();

        // =========== FINISH SCORING AND COLLECTING IN TOWER SIX ===========
        // scorer.set_intakes(0); // DELETE LATER
        scorer.wait_until_number_of_upper_balls_counted(1);
        scorer.set_flywheel(0);
        // scorer.wait_until_number_of_lower_balls_counted(2);
    }


    void run_skills_end()
    {
        // =========== BACK OUT OF TOWER EIGHT ===========
        drivetrain.drive_to_point(13.22, -14.17, 258.26, 1, 1);

        // =========== TURN ON INTAKES AND INDEXERS ===========
        scorer.set_intakes(127);
        scorer.set_indexers(127);

        // =========== COLLECT BALL J ===========
        drivetrain.drive_to_point(1.59, -16.54, 257.37, 2, 2, NULL, 0, 500);

        // =========== COLLECT TO BALL K ===========
        drivetrain.drive_to_point(36.25, -40.44, 217.66, 2, 3, NULL, 0, 1500);
        drivetrain.drive_to_point(31.43, -47.65, 224.01, 2, 3, NULL, 0, 1500);
        // =========== DRIVE TO TOWER NINE===========
        drivetrain.drive_to_point(12.43, -64.53, 227.73, 2, 3, [](){scorer.set_indexers(-8); scorer.set_flywheel(-2);}, 6, 1750, true);

        // =========== DRIVE TO TOWER NINE ===========
        // drivetrain.drive_to_point(-13.18, -64.80, 135.18, 2, 3);

        scorer.reset_balls_counted();
        scorer.set_indexers(127);
        scorer.collect_n_balls(3, 127, true);
        // // =========== CENTER ON TOWER NINE ===========
        drivetrain.center_on_tower_with_bumper(226.95, false, 1000);
        scorer.set_flywheel(127);

        scorer.score_n_balls(2, 700, false);
        drivetrain.stop_drive_motors();



        // =========== SCORE AND COLLECT IN TOWER NINE ===========
        scorer.wait_until_number_of_lower_balls_counted(3);
        scorer.wait_until_number_of_upper_balls_counted(1);

        // scorer.set_indexers(0);

    

    }
*/