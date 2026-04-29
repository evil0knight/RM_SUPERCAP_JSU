# Claude Notes

## Referee Power Data In This Repo
`refereePowerLimit` and `energyRemain` are not generated locally by the STM32 control loop.

They are written in [Communication.cpp](D:/RM/xiang_gang_ke_ji/SuperCapacitorController-master(2)/NEWSuperCapacitorController-master/Core/Src/Communication.cpp#L98) from CAN packet `0x061`:

- `feedbackRefereePowerLimit` -> `refereePowerLimit`
- `feedbackRefereeEnergyBuffer` -> `energyRemain`

So if someone asks "is `refereePowerLimit` from CAN / competition level?", the answer is: yes in this repo it comes from CAN, and that CAN payload is expected to be produced upstream from referee-system match data.

## `updateEnergy()` Reading Guide
The intent of [PowerManager.cpp](D:/RM/xiang_gang_ke_ji/SuperCapacitorController-master(2)/NEWSuperCapacitorController-master/Core/Src/PowerManager.cpp#L868) is:

1. Track changes in external `refereePowerLimit`.
2. Use `energyRemain` as the available power-buffer state.
3. Convert that state into `baseRefereePower`.
4. Copy `baseRefereePower` into `targetRefereePower`.
5. Let `updateVIP()` use that target to control A-side power flow with the supercapacitor.

In plain language:

- buffer/energy high -> allow more aggressive power use
- buffer/energy low -> reduce target referee-side power
- buffer/energy very low -> force conservative behavior to avoid over-limit punishment

## RM Rule Context
Official RoboMaster material describes the supercapacitor as a power buffer unit that stores energy during low chassis consumption and releases it during high demand.

Current official materials also show that modern seasons are no longer only the older `buffer_energy` model:

- the 2025 referee protocol still contains realtime `chassis_power_limit` and `buffer_energy`
- newer 2025/2026 rules and engine notes also include robot/chassis energy mechanisms
- 2025 RMUC materials mention bottom/chassis energy and wireless charging related mechanisms

Because of that, this repository should be read as implementing a classic buffer-energy power-control strategy, not as a full mirror of the latest RMUC energy rules.

## Reference Links
- https://www.robomaster.com/en-US/resource/pages/announcement/1768
- https://www.robomaster.com/zh-CN/products/components/detail/6145
- https://www.robomaster.com/zh-CN/resource/pages/activities/1004
