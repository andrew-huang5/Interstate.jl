function wrap(θ)
    mod(θ += pi, 2*pi) - pi
end

function clip(x, l)
    max(-l, min(l, x))
end

function keyboard_controller(KEY::Channel, 
                             CMD::Channel, 
                             SENSE::Channel, 
                             EMG::Channel;
                             K1=5, 
                             K2=.5, 
                             disp=false, 
                             V=0.0, 
                             θ = 0.0, 
                             V_max = 100.0, 
                             θ_step=0.1, 
                             V_step = 1.5)
    println("Keyboard controller in use.")
    println("Press 'i' to speed up,")
    println("Press 'k' to slow down,")
    println("Press 'j' to turn left,")
    println("Press 'l' to turn right.")

    while true
        sleep(0)
        @return_if_told(EMG)
        
        key = @take_or_default(KEY, ' ')
        meas = @fetch_or_continue(SENSE)

        speed = meas.speed
        heading = meas.heading
        segment = meas.road_segment_id
        if key == 'i'
            V = min(V_max, V+V_step)
        elseif key == 'j' 
            θ += θ_step
        elseif key == 'k'
            V = max(0, V-V_step)
        elseif key == 'l'
            θ -= θ_step
        end
        err_1 = V-speed
        err_2 = clip(θ-heading, π/2)
        cmd = [K1*err_1, K2*err_2]
        @replace(CMD, cmd)
        if disp
            print("\e[2K")
            print("\e[1G")
            @printf("Command: %f %f, speed: %f, segment: %d", cmd..., speed, segment)
        end
    end
end

function controller(CMD::Channel, 
                    SENSE::Channel, 
                    SENSE_FLEET::Channel, 
                    EMG::Channel,
                    road)
    
    println("Autonomous controller in use.")
    ego_meas = fetch(SENSE)
    fleet_meas = fetch(SENSE_FLEET)
    K₁ = K₂ = 0.5

    #change target velocity
    #variable for target lane
    max_velocity = 60
    target_velocity = 30
    target_lane = 2

    while true
        sleep(0.0)
        @return_if_told(EMG)
        
        ego_meas = @fetch_or_default(SENSE, ego_meas)
        fleet_meas = @fetch_or_default(SENSE_FLEET, fleet_meas)

        smallest_angle_1 = 2π
        smallest_angle_2 = 2π       
        smallest_angle_3 = 2π

        largest_angle = -2π
        local closest_car_front_1
        local closest_car_front_2
        local closest_car_front_3
        local closest_car_behind
        local target_car

        # find distances between cars
        for (id, m) ∈ fleet_meas
            center_position = road.segments[1].center
            ego_distance = ego_meas.position - center_position
            car_distance = m.position - center_position

            ego_angle = atan(ego_distance[2], ego_distance[1])
            car_angle = atan(car_distance[2], car_distance[1])

            difference = car_angle - ego_angle

            if(difference > 0 && difference < smallest_angle_1 && m.target_lane == 1)
                smallest_angle_1 = difference
                closest_car_front_1 = m
            end

            if(difference > 0 && difference < smallest_angle_2 && m.target_lane == 2)
                smallest_angle_2 = difference
                closest_car_front_2 = m
            end

            if(difference > 0 && difference < smallest_angle_3 && m.target_lane == 3)
                smallest_angle_3 = difference
                closest_car_front_3 = m
            end

            if(difference < 0 && difference > largest_angle)
                largest_angle = difference
                closest_car_behind = m
            end
        end

        # check if lane change is required
        if (closest_car_behind.speed > ego_meas.speed && (norm(ego_meas.position - closest_car_behind.position) < 30))
            target_car = get_target_car(ego_meas, closest_car_front_1, closest_car_front_2, closest_car_front_3)
        else
            target_car = follow_car(ego_meas, closest_car_front_1, closest_car_front_2, closest_car_front_3)
        end

        target_velocity = target_car.speed
        # print(ego_meas.speed, ' ' , target_velocity,' ', '\n')
        
        seg = road.segments[1]
        cte, ctv = get_crosstrack_error(ego_meas.position, ego_meas.heading, ego_meas.speed, target_car.target_lane, seg, road.lanes, road.lanewidth)
        δ = -K₁*cte-K₂*ctv
        if(norm(ego_meas.position - target_car.position) < 30)
            #print(norm(ego_meas.position - target_car.position))
            command = [(target_velocity - ego_meas.speed), max(min(δ, π/4.0), -π/4.0)]
        else
            # can use 1 for the change in velocity if you want car to speed up
            command = [1 max(min(δ, π/4.0), -π/4.0)]  
        end
        # command = [0 max(min(δ, π/4.0), -π/4.0)]
        @replace(CMD, command)
        
    end

end

# determine which car to switch to
function get_target_car(ego, car1, car2, car3)
    distance1 = norm(ego.position - car1.position)
    distance2 = norm(ego.position - car2.position)
    distance3 = norm(ego.position - car3.position)

    if (ego.target_lane == car1.target_lane)
        if (distance2 >= distance3)
            return car2
        elseif (distance3 > distance2)
            return car3
        else 
            return car1
        end
    elseif (ego.target_lane == car2.target_lane)
        if (distance1 >= distance3)
            return car1
        elseif (distance3 > distance1)
            return car3
        else 
            return car2
        end
    else
        if (distance1 >= distance2)
            return car1
        elseif (distance2 > distance1)
            return car1
        else 
            return car3
        end
    end
    #= if(distance1 >= distance2 && distance1 >= distance3 && ego.target_lane != car1.target_lane)
        return car1
    elseif(distance2 >= distance1 && distance2 >= distance3 && ego.target_lane != car2.target_lane)
        return car2
    else 
        return car3
    end =#
end

# follow car in lane
function follow_car(ego, car1, car2, car3)
    if (ego.target_lane == car1.target_lane)
        return car1
    elseif (ego.target_lane == car2.target_lane)
        return car2
    else
        return car3
    end
end

