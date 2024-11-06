# in general
 * column generation is complete, but slow
 * in practice, we want very fast (greedy) algorithm for real-time control
   (and possibly for stability/reliability/simplicity!, like, you don't want
   the plan to change because the colgen was slightly worse on some instance.)
 * stability and/or first-action is a big issue (changing the first action
   often is typically a sign that somethign is wrong)
 * when estimates are not perfect (which they will not be), we can get 
   oscillating plans when values change around some threshold values.
   stabilizing terms are necessary.

## greedy planning
 * greedily deciding the airborne drones first is obviously not so good
 * but greedily deciding each drone on the ground might be a much better
   model. What about tree-searching on the airbone drones only!?

 * It's not a good idea to plan the airborne drones first,
   because it will always prioritise having them in the air for
   as long as possible. If there are other drones taking over 
   the other tasks, then those could have higher overall score
   and be chosen first, and it will become better for the 
   airborne drone to land, even if there is a little battery left.

   This also means that time-discounting in a good way can be important!


