//Multi tracker.cpp 


void collect_analysis(tAnnInfo* pCurrFrameBBs, tAnnInfo* pPrevFrameBBs, tLanesInfo* pLanesInfo)
{
    tAnnInfo* pBBNode = NULL;
    tAnnInfo* pCurrBB = NULL;
    static int framecnt=0;
    tAnnInfo* ptmp = NULL;
    tAnnInfo* ctmp = pCurrFrameBBs;

    if(!pCurrFrameBBs || !pPrevFrameBBs || !pLanesInfo)
        return;

    // revathy - Arrival rate calculation
    ++framecnt;
    LOGD("\n Frames so far:%d", framecnt);
    if(framecnt > 30)
    {
      tLane* lntmp = pLanesInfo->pLanes;
      framecnt =0;
      while(lntmp)
      {
        LOGD("\n Arrival Rate at lane %d is %d", lntmp->nLaneId, lntmp->nArrivalRate);
        lntmp->nArrivalRate = 0;
        lntmp->nDepartureRate=0;
        lntmp = lntmp->pNext;
      }
    }

    /* Count new objects in current frame*/
    tLane* lntmp = NULL;
    while(ctmp)
    {
      if((!(ptmp=getBBById(pPrevFrameBBs, ctmp->nBBId))))
      {
        ctmp->bIsTracked = false;
        //LOGD("New obj detected\n");
        if(lntmp = laneWithThisBB(pLanesInfo, ctmp) && (ctmp->nLaneId ==INVALID_LANE_ID ))
        {
          //LOGD("BB in lane\n");
          ++(lntmp->nArrivalRate);
        }
      }
      else
      {
        ctmp->bIsTracked = true;
      }
      ctmp = ctmp->pNext;
    }

    //Unni - wait time calc
    pBBNode = pPrevFrameBBs;
    while(pBBNode)
    {
        if((pCurrBB = getBBById(pCurrFrameBBs, pBBNode->nBBId)))
        {
            /** an object is tracked in the curr frame;
             * get the laneid of this tracked object and compare with the previous frame lane id.
             * If the laneid is same, do nothing . else, check for valid lane change from a table indicating the configuration of valid lane changes
             * if not valid lane change, do error compensation / ignore counting
             *
             *
             */
            tLane* pLPrev = laneWithThisBB(pLanesInfo, pBBNode); //-??
            tLane* pLCurr = laneWithThisBB(pLanesInfo, pCurrBB);

            if(pBBNode->nLaneId == INVALID_LANE_ID)   /*assign prev BB's basic detail here - if its not already populated --?? To check*/
            {
                pBBNode->fStartTS = pBBNode->fCurrentFrameTimeStamp;
                pBBNode->nLaneId = pLPrev ? pLPrev->nLaneId : INVALID_LANE_ID;
            }

            pCurrBB->nLaneId = pLCurr ? pLCurr->nLaneId : INVALID_LANE_ID;
            pCurrBB->nLaneHistory = pBBNode->nLaneHistory;

            if(pBBNode->nLaneId != pCurrBB->nLaneId)
            {
              if(pCurrBB->nLaneId == INVALID_LANE_ID) /* Valid to invalid lanechange, so must be crossing the intersection*/
              {
                ++(pLCurr->nDepartureRate);        
              }
              else if((pBBNode->nLaneId == INVALID_LANE_ID) && (isValidRoute(pCurrBB->nLaneHistory,pCurrBB->nLaneId)))
              /* Invalid to valid lanechange, so must enter a new lane*/
              {
                ++(pLCurr->nArrivalRate);



              }
            }
            else
            {


            }

            }








            {


                if(pBBNode->nLaneId != pCurrBB->nLaneId)
                {
                    pCurrBB->nLaneHistory = pBBNode->nLaneId;
                    if(pCurrBB->nLaneId != INVALID_LANE_ID && pBBNode->nLaneHistory != INVALID_LANE_ID)
                    {
                        #if 0
                        LOGV("we have route flux from lane %d to %d %d\n", pBBNode->nLaneHistory, pCurrBB->nLaneId, pCurrBB->nClassId);
                        LOGV("deref ppRouteTrafficInfo[]=%p\n", pLanesInfo->ppRouteTrafficInfo[pBBNode->nLaneHistory]);
                        LOGV("deref ppRouteTrafficInfo[][]=%p\n", &pLanesInfo->ppRouteTrafficInfo[pBBNode->nLaneHistory][pCurrBB->nLaneId]);
                        LOGV("deref ppRouteTrafficInfo[][]=%p\n", pLanesInfo->ppRouteTrafficInfo[pBBNode->nLaneHistory][pCurrBB->nLaneId].pnVehicleCount);
                        LOGV("deref ppRouteTrafficInfo[][]=%lld\n", pLanesInfo->ppRouteTrafficInfo[pBBNode->nLaneHistory][pCurrBB->nLaneId].pnVehicleCount[pCurrBB->nClassId]);
                        #endif
                        (pLanesInfo->ppRouteTrafficInfo[pBBNode->nLaneHistory][pCurrBB->nLaneId].pnVehicleCount[pCurrBB->nClassId])++;
                        if(isViolation(pBBNode->nLaneHistory, pCurrBB->nLaneId))
                        {
                            LOGV("violation @ %f; BBID=%d type=%s\n", pCurrBB->fCurrentFrameTimeStamp, pCurrBB->nBBId, pCurrBB->pcClassName);
                        }
                    }
                    if(pLPrev)
                        pLPrev->pnVehicleCount[pCurrBB->nClassId]++;
                    /** object now exited the lane */
                    double fDurationOfStayInThisLane = pCurrBB->fCurrentFrameTimeStamp - pCurrBB->fStartTS;
                    if(pLPrev)
                    {
                        pLPrev->fTotalStayDuration += fDurationOfStayInThisLane;
                        pCurrBB->fStartTS = pCurrBB->fCurrentFrameTimeStamp;
                        pLPrev->nTotalVehiclesSoFar = 0;
                        for(int i = 0; i < pLPrev->nTypes; i++)
                            pLPrev->nTotalVehiclesSoFar += pLPrev->pnVehicleCount[i];
                        pLPrev->fAvgStayDuration = pLPrev->nTotalVehiclesSoFar ? pLPrev->fTotalStayDuration / (pLPrev->nTotalVehiclesSoFar) : 0;
                        LOGV("lane %d; fAvgStayDuration=%f nTotalVehiclesSoFar=%lld\n", pLPrev->nLaneId, pLPrev->fAvgStayDuration, pLPrev->nTotalVehiclesSoFar);
                    }
                }
                else
                {
                    pCurrBB->fStartTS = pBBNode->fStartTS;
                }
            }
        }
        else
        {
            /** the object went out of scene */
            /** counting vehicle types: */
            /** check which lane the vehicle belonged to, and increment corresponding count */
            tLane* pL;
            if(pBBNode->fStartTS /**< count this only if we tracked same obj over atleast 2 frames */
                && (pL = laneWithThisBB(pLanesInfo, pBBNode)))
            {
            }
            else
            {
                LOGV("we couldn't identify lane; \n");
            }
        }
        pBBNode = pBBNode->pNext;
    }

    return;
}

bool isValidRoute(int prevLane, int currLane)
{
  int nlndiff = abs(prevLane,currLane);
  if(nlndiff == 1 || nlndiff == 0 )
  {
    return true;
  }
  else
  {
    return false;
  }
}
