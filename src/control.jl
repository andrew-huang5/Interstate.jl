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

    while true
        sleep(0.0)
        @return_if_told(EMG)
        
        ego_meas = @fetch_or_default(SENSE, ego_meas)
        fleet_meas = @fetch_or_default(SENSE_FLEET, fleet_meas)
        target_velocity = 1
        target_lane = 1
        
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

        front = ego_meas.position[2] + ego_meas.front
        # for (id, m) ∈ fleet_meas
        #     closest_position = INT_MAX
        #     distance = m.position[1] + m.rear - front
        #     if (m.target_lane == target_lane && distance > front)
        #         closest_position = 


        #     # seg = road.segments[m.road_segment_id]
        #     # cte, ctv = get_crosstrack_error(m.position, m.heading, m.speed, m.target_lane, seg, road.lanes, road.lanewidth)
        #     # δ = -K₁*cte-K₂*ctv
        #     # command = [0.0 max(min(δ, π/4.0), -π/4.0)]
        #     # @replace(CMD[id], command)
        # end
        
        command = [0.0, 0.0]
        seg = road.segments[1]
        cte, ctv = get_crosstrack_error(ego_meas.position, ego_meas.heading, ego_meas.speed, ego_meas.target_lane, seg, road.lanes, road.lanewidth)
        δ = -K₁*cte-K₂*ctv
        command = [0.0 max(min(δ, π/4.0), -π/4.0)]
        @replace(CMD, command)
        
    end

end


# how to stay in one lane
# what is in ego_meas?
# using get_crosstrack_error
