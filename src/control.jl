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
    local target_car = undef
    local old_target = undef
    counter = 1000
    change_lanes = true

    while true
        sleep(0.0)
        @return_if_told(EMG)
        
        ego_meas = @fetch_or_default(SENSE, ego_meas)
        fleet_meas = @fetch_or_default(SENSE_FLEET, fleet_meas)

        smallest_angle_1 = 2π
        smallest_angle_2 = 2π       
        smallest_angle_3 = 2π

        largest_angle = -2π

        closest_car_front_1 = undef
        closest_car_front_2 = undef
        closest_car_front_3 = undef

        closest_car_behind = undef

        center_position = road.segments[1].center
        ego_distance = ego_meas.position - center_position
        ego_angle = atan(ego_distance[2], ego_distance[1])

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

            if(difference < 0 && difference > largest_angle && m.target_lane == ego_meas.target_lane)
                largest_angle = difference
                closest_car_behind = m
                # println("CAR FOUND")
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

        for (id, m) ∈ fleet_meas
            car_distance = m.position - center_position
            car_angle = atan(car_distance[2], car_distance[1])

            if ego_meas.target_lane != m.target_lane && m.target_lane == target_lane
                car_distance = m.position - center_position
                car_angle = atan(car_distance[2], car_distance[1])

                # println(car_angle)
                # println(ego_angle)

                difference = wrap(car_angle - ego_angle)

                if difference < .1 && difference > -.1 # norm(ego_meas.position - m.position) < 20
                    change_lanes = false
                    println("DONT CHANGE LANES!")
                    counter = 0
                    println(m.target_lane)
                    target_car = old_target
                    target_lane = old_target.target_lane
                end
            elseif abs(ego_meas.target_lane - target_lane) == 2 && m.target_lane == 2
                if difference < .1 && difference > -.1 # && norm(ego_meas.position - m.position) < 20
                    change_lanes = false
                    println("cant change 2")
                    counter = 0
                    println(m.target_lane)
                    target_car = old_target
                    target_lane = old_target.target_lane
                end
            end
        end
        

        if counter == 1000
            counter = 0
            println("\nnew target car")
            old_target = target_car
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

            # if closest_car_behind != undef
            #     if norm(ego_meas.position - closest_car_behind.position) < 10 && closest_car_behind.speed > ego_meas.speed
            #         # print("INCOMING")
            #         if target_lane == 1 || target_lane == 2
            #             target_lane = target_lane + 1
            #         else
            #             target_lane = 1
            #         end
            #     end
            # end



            if target_car == undef
                target_velocity = 50
            else
                target_velocity = target_car.speed
            end

            if !change_lanes
                change_lanes = true
                target_lane = old_target.target_lane
                target_car = old_target
            end
            
            seg = road.segments[1]
            cte, ctv = get_crosstrack_error(ego_meas.position, ego_meas.heading, ego_meas.speed, target_lane, seg, road.lanes, road.lanewidth)
            δ = -K₁*cte-K₂*ctv
            if(target_car != undef && norm(ego_meas.position - target_car.position) < 35)
                # println("SLOWING DOWN")
                command = [(target_velocity - ego_meas.speed), max(min(δ, π/4.0), -π/4.0)]
            else
                ego_meas.speed <= 25 ? command = [.8 max(min(δ, π/4.0), -π/4.0)]  : command = [0 max(min(δ, π/4.0), -π/4.0)]  
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
                ego_meas.speed <= 25 ? command = [.8 max(min(δ, π/4.0), -π/4.0)]  : command = [0 max(min(δ, π/4.0), -π/4.0)]  
            end
            # command = [0 max(min(δ, π/4.0), -π/4.0)]
            @replace(CMD, command)
        end
        counter += 1
    end

end


# how to stay in one lane
# what is in ego_meas?
# using get_crosstrack_error

function get_target_car(ego, car1, car2, car3)
    # if car1 != undef
    #     distance1 = norm(ego.position - car1.position)
    # else
    #     distance1 = 100000000000
    #     print("HIIII")
    # end

    # if car2 != undef
    #     distance2 = norm(ego.position - car2.position)
    # else
    #     distance2 = 100000000000
    #     print("HIIII")
    # end
    
    # if car3 != undef
    #     distance3 = norm(ego.position - car3.position)
    # else
    #     distance3 = 100000000000
    #     print("HIIII")
    # end

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
