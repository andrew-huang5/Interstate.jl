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
    target_velocity = 30
    target_lane = 2
    target_car = undef
    counter = 100

    while true
        sleep(0.0)
        @return_if_told(EMG)
        
        ego_meas = @fetch_or_default(SENSE, ego_meas)
        fleet_meas = @fetch_or_default(SENSE_FLEET, fleet_meas)

        smallest_angle_1 = 2π
        smallest_angle_2 = 2π       
        smallest_angle_3 = 2π

        largest_angle_1 = -2π
        largest_angle_2 = -2π
        largest_angle_3 = -2π

        closest_car_front_1 = undef
        closest_car_front_2 = undef
        closest_car_front_3 = undef

        closest_car_behind_1 = undef
        closest_car_behind_2 = undef
        closest_car_behind_3 = undef

        center_position = road.segments[1].center
        ego_distance = ego_meas.position - center_position
        ego_angle = atan(ego_distance[2], ego_distance[1])
        
        # finding closest cars
        for (id, m) ∈ fleet_meas
            car_distance = m.position - center_position

            car_angle = atan(car_distance[2], car_distance[1])

            difference = wrap(car_angle - ego_angle)

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

            if(difference < 0 && difference > largest_angle_1 && m.target_lane == 1)
                largest_angle_1 = difference
                closest_car_behind_1 = m
            end
            if(difference < 0 && difference > largest_angle_2 && m.target_lane == 2)
                largest_angle_2 = difference
                closest_car_behind_2 = m
            end
            if(difference < 0 && difference > largest_angle_3 && m.target_lane == 3)
                largest_angle_3 = difference
                closest_car_behind_3 = m
            end
        end

        if target_lane == 1 && closest_car_front_1 != target_car
            target_car = closest_car_front_1
        end
        if target_lane == 2 && closest_car_front_2 != target_car
            target_car = closest_car_front_2
        end
        if target_lane == 3 && closest_car_front_3 != target_car
            target_car = closest_car_front_3
        end

        if counter == 100
            counter = 0
            curr_lane = target_lane
            target_car = get_target_car(ego_meas, closest_car_front_1, closest_car_front_2, closest_car_front_3)
            
            if closest_car_front_1 == undef
                target_lane = 1
            elseif closest_car_front_2 == undef
                target_lane = 2
            elseif closest_car_front_3 == undef
                target_lane = 3
            else
                target_lane = target_car.target_lane
            end

            if target_car == undef
                target_velocity = 60
            else
                target_velocity = target_car.speed
            end

            temp = lane_change(ego_meas, closest_car_front_1, closest_car_front_2, closest_car_front_3, closest_car_behind_1, closest_car_behind_2, closest_car_behind_3, curr_lane, road.lanewidth, road.segments[1].radius, center_position)
            if temp == 0
                print("stay")
                target_lane = curr_lane
            end

            seg = road.segments[1]
            cte, ctv = get_crosstrack_error(ego_meas.position, ego_meas.heading, ego_meas.speed, target_lane, seg, road.lanes, road.lanewidth)
            δ = -K₁*cte-K₂*ctv
            if(target_car != undef && norm(ego_meas.position - target_car.position) < 35)
                # println("SLOWING DOWN")
                command = [(target_velocity - ego_meas.speed), max(min(δ, π/4.0), -π/4.0)]
            else
                ego_meas.speed <= 35 ? command = [.8 max(min(δ, π/4.0), -π/4.0)]  : command = [0 max(min(δ, π/4.0), -π/4.0)]  
            end
            # command = [0 max(min(δ, π/4.0), -π/4.0)]
            @replace(CMD, command)
        else
            seg = road.segments[1]
            cte, ctv = get_crosstrack_error(ego_meas.position, ego_meas.heading, ego_meas.speed, target_lane, seg, road.lanes, road.lanewidth)
            δ = -K₁*cte-K₂*ctv
            if(target_car != undef && norm(ego_meas.position - target_car.position) < 35)
                # println("SLOWING DOWN")
                command = [(target_velocity - ego_meas.speed), max(min(δ, π/4.0), -π/4.0)]
            else
                ego_meas.speed <= 35 ? command = [.8 max(min(δ, π/4.0), -π/4.0)]  : command = [0 max(min(δ, π/4.0), -π/4.0)]  
            end
            # command = [0 max(min(δ, π/4.0), -π/4.0)]
            @replace(CMD, command)
        end
        counter += 1
    end

end

function get_target_car(ego, car1, car2, car3)
    if car1 == undef
        # print("car1")
        return undef
    end

    if car2 == undef
        # print("car2")
        return undef
    end

    if car3 == undef
        # print("car3")
        return undef
    end

    distance1 = norm(ego.position - car1.position)
    distance2 = norm(ego.position - car2.position)
    distance3 = norm(ego.position - car3.position)

    if(distance1 >= distance2 && distance1 >= distance3)
        return car1
    elseif(distance2 >= distance1 && distance2 >= distance3)
        return car2
    else 
        return car3
    end
end

function lane_change(ego, car1, car2, car3, car4, car5, car6, lane, width, radius, center)
    ego_distance = ego.position - center
    ego_angle = atan(ego_distance[2],ego_distance[1])
    if (lane == 1)
        # calculate gap
        car_distance = car2.position - center
        car_angle = atan(car_distance[2], car_distance[1])
        car_distance2 = car5.position - center
        car_angle2 = atan(car_distance2[2], car_distance2[1])
        difference = wrap(car_angle - car_angle2)
        length = (radius+width)*difference

        # calculate angle differences between cars
        diff1 = wrap(car_angle - ego_angle)
        diff2 = wrap(ego_angle - car_angle2)
        
        if diff1 < .1 || diff2 < .1 || length <= ego.front*2
            print("no change 1 \n")
            return 0
        else
            return 1
        end
    elseif (lane == 2)
        # calculate gap
        car_distance = car1.position - center
        car_angle = atan(car_distance[2], car_distance[1])
        car_distance2 = car4.position - center
        car_angle2 = atan(car_distance2[2], car_distance2[1])
        difference1 = wrap(car_angle - car_angle2)
        length1 = (radius+width)*difference1
        
        car_distance3 = car3.position - center
        car_angle3 = atan(car_distance3[2], car_distance3[1])
        car_distance4 = car6.position - center
        car_angle4 = atan(car_distance4[2], car_distance4[1])
        difference2 = wrap(car_angle3 - car_angle4)
        length2 = (radius+width)*difference2

        # calculate angle differences between cars
        diff1 = wrap(car_angle - ego_angle)
        diff2 = wrap(ego_angle - car_angle2)
        diff3 = wrap(car_angle3 - ego_angle)
        diff4 = wrap(ego_angle - car_angle4)

        if (diff1 < .1 || diff2 < .1 || length1 <= ego.front*2) && (diff3 < .1 || diff4 < .1 || length2 <= ego.front*2)
            print("no change 2 \n")
            return 0
        else
            return 1
        end
    elseif (lane == 3)
        # calculate gap
        car_distance = car3.position - center
        car_angle = atan(car_distance[2], car_distance[1])
        car_distance2 = car6.position - center
        car_angle2 = atan(car_distance2[2], car_distance2[1])
        difference = wrap(car_angle - car_angle2)
        length = (radius+width)*difference

        # calculate angle differences between cars
        diff1 = wrap(car_angle - ego_angle)
        diff2 = wrap(ego_angle - car_angle2)
        
        if diff1 < .1 || diff2 < .1 || length <= ego.front*2
            print("no change 3 \n")
            return 0
        else
            return 1
        end
    else
        print("check \n")
        return 1
    end
end


