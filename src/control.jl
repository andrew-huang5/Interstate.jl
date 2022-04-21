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
    target_velocity = 1
    target_lane = 2

    while true
        sleep(0.0)
        @return_if_told(EMG)
        
        ego_meas = @fetch_or_default(SENSE, ego_meas)
        fleet_meas = @fetch_or_default(SENSE_FLEET, fleet_meas)
        
        # println(ego_meas)

        # for meas in ego_meas
        #     print(meas)
        # end

        # for m ∈ ego_meas
        #     println(m.road_segment_id)
        #     seg = road.segments[m.road_segment_id]
        #     cte, ctv = get_crosstrack_error(m.position, m.heading, m.speed, m.target_lane, seg, road.lanes, road.lanewidth)
        #     δ = -K₁*cte-K₂*ctv
        #     command = [0.0 max(min(δ, π/4.0), -π/4.0)]
        #     # @replace(CMD[id], command)
        # end

        # println(ego_meas.road_segment_id) # returns -1

        # ego_car_front = ego_meas.position[2] + ego_meas.front
        # closest_position = 10000000000000
        # print(fleet_meas)
        # for (id, m) ∈ fleet_meas
        #     front_car_rear = m.position[2] + m.rear
        #     if (m.target_lane == target_lane && front_car_rear > ego_car_front && closest_position >= front_car_rear)
        #         closest_position = front_car_rear
        #         if(front_car_rear - ego_car_front <= .5)
        #             target_velocity = 0
        #         else
        #             target_velocity = m.target_vel
        #         end
        #     end

        #     # seg = road.segments[m.road_segment_id]
        #     # cte, ctv = get_crosstrack_error(m.position, m.heading, m.speed, m.target_lane, seg, road.lanes, road.lanewidth)
        #     # δ = -K₁*cte-K₂*ctv
        #     # command = [0.0 max(min(δ, π/4.0), -π/4.0)]
        #     # @replace(CMD[id], command)
        # end

        smallest_angle = 2π
        local closest_car


        for (id, m) ∈ fleet_meas
            if(m.target_lane == target_lane)
                center_position = road.segments[1].center
                ego_distance = ego_meas.position - center_position
                car_distance = m.position - center_position

                ego_angle = atan(ego_distance[2], ego_distance[1])
                car_angle = atan(car_distance[2], car_distance[1])

                difference = car_angle - ego_angle

                if(difference > 0 && difference < smallest_angle)
                    smallest_angle = difference
                    closest_car = m
                end
            end
        end
        target_velocity = closest_car.speed
        # print(ego_meas.speed, ' ' , target_velocity,' ', '\n')
        
        command = [1, 0.0]
        seg = road.segments[1]
        cte, ctv = get_crosstrack_error(ego_meas.position, ego_meas.heading, ego_meas.speed, ego_meas.target_lane, seg, road.lanes, road.lanewidth)
        δ = -K₁*cte-K₂*ctv
        if(norm(ego_meas.position - closest_car.position) < 30)
            print(norm(ego_meas.position - closest_car.position))
            command = [(target_velocity - ego_meas.speed), max(min(δ, π/4.0), -π/4.0)]
        else
            command = [0 max(min(δ, π/4.0), -π/4.0)]  
        end
        # command = [0 max(min(δ, π/4.0), -π/4.0)]
        @replace(CMD, command)
        
    end

end


# how to stay in one lane
# what is in ego_meas?
# using get_crosstrack_error
