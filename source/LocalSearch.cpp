#include <algorithm>
#include <cmath>
#include <vector>
#include <set>
#include <iostream>
#include "LocalSearch.h"
#include "Individual.h"
#include "CircleSector.h"
#include "Params.h"

bool operator==(const TimeWindowData& twData1, const TimeWindowData& twData2)
{
	return twData1.firstNodeIndex == twData2.firstNodeIndex &&
		twData1.lastNodeIndex == twData2.lastNodeIndex &&
		twData1.duration == twData2.duration &&
		twData1.timeWarp == twData2.timeWarp &&
		twData1.earliestArrival == twData2.earliestArrival &&
		twData1.latestArrival == twData2.latestArrival;
}

bool cmpd(double a, double b, double eps = 1e-5)
{
	return std::fabs(a - b) < eps;
}

void LocalSearch::initializeConstruction(Individual* indiv, std::vector<NodeToInsert>* nodesToInsert)
{
	// Initialize datastructures relevant for constructions.
	// Local search-related data structures are not initialized.
	emptyRoutes.clear();
	TimeWindowData depotTwData;
	depotTwData.firstNodeIndex = 0;
	depotTwData.lastNodeIndex = 0;
	depotTwData.duration = 0;
	depotTwData.timeWarp = 0;
	depotTwData.earliestArrival = params->cli[0].earliestArrival;
	depotTwData.latestArrival = params->cli[0].latestArrival;
	// Initializing time window data for clients
	for (int i = 1; i <= params->nbClients; i++)
	{
		TimeWindowData* myTwData = &clients[i].twData;
		myTwData->firstNodeIndex = i;
		myTwData->lastNodeIndex = i;
		myTwData->duration = params->cli[i].serviceDuration;
		myTwData->earliestArrival = params->cli[i].earliestArrival;
		myTwData->latestArrival = params->cli[i].latestArrival;
	}


	// Initialize routes
	for (int r = 0; r < params->nbVehicles; r++)
	{
		Node* myDepot = &depots[r];
		Node* myDepotFin = &depotsEnd[r];
		myDepot->prev = myDepotFin;
		myDepotFin->next = myDepot;
		myDepot->next = myDepotFin;
		myDepotFin->prev = myDepot;

		myDepot->twData = depotTwData;
		myDepot->prefixTwData = depotTwData;
		myDepot->postfixTwData = depotTwData;

		myDepotFin->twData = depotTwData;
		myDepotFin->prefixTwData = depotTwData;
		myDepotFin->postfixTwData = depotTwData;

		updateRouteData(&routes[r]);
	}


	// Initialize clients.
	for (int i = 1; i <= params->nbClients; i++)
	{
		NodeToInsert nodeToInsert;
		nodeToInsert.clientIdx = i;
		nodeToInsert.twData = clients[i].twData;
		nodeToInsert.load = params->cli[i].demand;
		nodeToInsert.angleFromDepot = atan2(params->cli[i].coordY - params->cli[0].coordY, params->cli[i].coordX - params->cli[0].coordX);
		nodeToInsert.serviceDuration = params->cli[i].serviceDuration;
		nodesToInsert->push_back(nodeToInsert);
	}


}

void LocalSearch::constructIndividualBySweep(int fillPercentage, Individual* indiv)
{
	std::vector<NodeToInsert> nodesToInsert;
	initializeConstruction(indiv, &nodesToInsert);

	std::vector< std::vector< int > > nodeIndicesPerRoute;

	// Sort nodes according to angle with depot.
	std::sort(std::begin(nodesToInsert),
		std::end(nodesToInsert),
		[](NodeToInsert a, NodeToInsert b) {return a.angleFromDepot < b.angleFromDepot; });

	// Distribute clients over routes.
	int load = 0;
	std::vector< int > nodeIndicesInRoute;
	for (int i = 0; i < static_cast<int>(nodesToInsert.size()); i++)
	{
		if (load > 0 && load + nodesToInsert[i].load > fillPercentage * params->vehicleCapacity / 100 && nodeIndicesPerRoute.size() + 1 < routes.size())
		{
			nodeIndicesPerRoute.push_back(nodeIndicesInRoute);
			nodeIndicesInRoute.clear();
			load = 0;
		}

		load += nodesToInsert[i].load;
		nodeIndicesInRoute.push_back(i);
	}

	nodeIndicesPerRoute.push_back(nodeIndicesInRoute);

	// Construct routes
	for (int r = 0; r < static_cast<int>(nodeIndicesPerRoute.size()); r++)
	{
		int depotOpeningDuration = depots[r].twData.latestArrival - depots[r].twData.earliestArrival;
		std::vector<int> nodeIndicesToInsertShortTw;
		std::vector<int> nodeIndicesToInsertLongTw;
		for (int idx : nodeIndicesPerRoute[r])
		{
			// Arbitrary division, but for all instances time windows are either much shorter than
			// half of depotOpeningDuration, or much larger.
			if ((nodesToInsert[idx].twData.latestArrival - nodesToInsert[idx].twData.earliestArrival) * 2 > depotOpeningDuration)
				nodeIndicesToInsertLongTw.push_back(idx);
			else
				nodeIndicesToInsertShortTw.push_back(idx);
		}

		// Sort routes with short time window in increasing end of time window.
		std::sort(std::begin(nodeIndicesToInsertShortTw),
			std::end(nodeIndicesToInsertShortTw),
			[&nodesToInsert](int a, int b) { return nodesToInsert[a].twData.latestArrival < nodesToInsert[b].twData.latestArrival; });

		// Insert nodes with short time window in order in the route.
		Node* prev = routes[r].depot;
		for (int i = 0; i < static_cast<int>(nodeIndicesToInsertShortTw.size()); i++)
		{
			Node* toInsert = &clients[nodesToInsert[nodeIndicesToInsertShortTw[i]].clientIdx];
			Node* insertionPoint = prev;
			toInsert->prev = insertionPoint;
			toInsert->next = insertionPoint->next;
			insertionPoint->next->prev = toInsert;
			insertionPoint->next = toInsert;
			prev = toInsert;
		}

		updateRouteData(&routes[r]);

		// Insert remaining nodes according to best distance
		for (int i = 0; i < static_cast<int>(nodeIndicesToInsertLongTw.size()); i++)
		{
			double bestCost = std::numeric_limits<double>::max();
			Node* bestPred = nullptr;
			Node* prev = routes[r].depot;
			for (int j = 0; j <= routes[r].nbCustomers; j++)
			{
				// Compute insertion cost
				double insertionCost = params->timeCost.get(prev->cour, nodesToInsert[nodeIndicesToInsertLongTw[i]].clientIdx) +
					params->timeCost.get(nodesToInsert[nodeIndicesToInsertLongTw[i]].clientIdx, prev->next->cour) -
					params->timeCost.get(prev->cour, prev->next->cour);

				if (insertionCost < bestCost)
				{
					bestCost = insertionCost;
					bestPred = prev;
				}

				prev = prev->next;
			}

			Node* toInsert = &clients[nodesToInsert[nodeIndicesToInsertLongTw[i]].clientIdx];
			Node* insertionPoint = bestPred;
			toInsert->prev = insertionPoint;
			toInsert->next = insertionPoint->next;
			insertionPoint->next->prev = toInsert;
			insertionPoint->next = toInsert;
			updateRouteData(&routes[r]);
		}
	}

	// Register the solution produced by the construction heuristic in the individual.
	exportIndividual(indiv);
}

void LocalSearch::constructIndividualFromSplit(Individual* indiv, Individual randomIndiv)
{
	std::vector<NodeToInsert> nodesToInsert;
	initializeConstruction(indiv, &nodesToInsert);

	for (int r = 0; r < params->nbVehicles; r++){
		if (!randomIndiv.chromR[r].empty()){
			// Insert the first client in the route
			Node* prev = routes[r].depot;
			for (int i = 0; i < static_cast<int>(randomIndiv.chromR[r].size()); i++)
			{
				Node* toInsert = &clients[nodesToInsert[randomIndiv.chromR[r][i]->cour-1].clientIdx];
				Node* insertionPoint = prev;
				toInsert->prev = insertionPoint;
				toInsert->next = insertionPoint->next;
				insertionPoint->next->prev = toInsert;
				insertionPoint->next = toInsert;
				prev = toInsert;

			}
			updateRouteData(&routes[r]);
		}
	}
	// Register the solution produced by the construction heuristic in the individual.
	exportIndividual(indiv);
}




void LocalSearch::constructIndividualWithSeedOrder(int toleratedCapacityViolation, int toleratedTimeWarp,
	bool useSeedClientFurthestFromDepot, Individual* indiv)
{
	std::vector<NodeToInsert> nodesToInsert;
	initializeConstruction(indiv, &nodesToInsert);
	std::set<int> unassignedNodeIndices;
	for (int i = 1; i <= params->nbClients; i++)
	{
		unassignedNodeIndices.insert(i - 1);
	}
	// Construct routes
	for (int r = 0; r < static_cast<int>(routes.size()) && unassignedNodeIndices.size() > 0; r++)
	{
		// Note that if the seed client is the unassigned client closest to the depot, we do not
		// have to do any initialization and can just start inserting nodes that are best according
		// to distance in the main loop.
		if (useSeedClientFurthestFromDepot)
		{
			int furthestNodeIdx = -1;
			double furthestNodeCost = -1.0;
			for (int idx : unassignedNodeIndices)
			{
				double insertionCost = params->timeCost.get(routes[r].depot->cour, nodesToInsert[idx].clientIdx) +
					params->timeCost.get(nodesToInsert[idx].clientIdx, routes[r].depot->next->cour) -
					params->timeCost.get(routes[r].depot->cour, routes[r].depot->next->cour);

				if (insertionCost > furthestNodeCost)
				{
					furthestNodeCost = insertionCost;
					furthestNodeIdx = idx;
				}
			}
			Node* toInsert = &clients[nodesToInsert[furthestNodeIdx].clientIdx];
			toInsert->prev = routes[r].depot;
			toInsert->next = routes[r].depot->next;
			routes[r].depot->next->prev = toInsert;
			routes[r].depot->next = toInsert;
			updateRouteData(&routes[r]);
			unassignedNodeIndices.erase(furthestNodeIdx);
		}

		bool insertedNode = true;
		while (insertedNode)
		{
			insertedNode = false;
			double bestCost = std::numeric_limits<double>::max();
			Node* bestPred = nullptr;
			int bestNodeIdx;
			for (int idx : unassignedNodeIndices)
			{
				// Do not allow insertion if capacity is exceeded more than tolerance.
				if (routes[r].load + nodesToInsert[idx].load > params->vehicleCapacity + toleratedCapacityViolation)
					continue;

				Node* prev = routes[r].depot;
				for (int j = 0; j <= routes[r].nbCustomers; j++)
				{
					// Do not allow insertions if time windows are violated more than tolerance
					TimeWindowData routeTwData =
						MergeTWDataRecursive(prev->prefixTwData, nodesToInsert[idx].twData, prev->next->postfixTwData);
					if (routeTwData.timeWarp > toleratedTimeWarp)
					{
						prev = prev->next;
						continue;
					}

					// Compute insertion cost
					double insertionCost = params->timeCost.get(prev->cour, nodesToInsert[idx].clientIdx) +
						params->timeCost.get(nodesToInsert[idx].clientIdx, prev->next->cour) -
						params->timeCost.get(prev->cour, prev->next->cour);

					if (insertionCost < bestCost)
					{
						bestCost = insertionCost;
						bestPred = prev;
						bestNodeIdx = idx;
					}

					prev = prev->next;
				}
			}
			if (bestCost < std::numeric_limits<double>::max())
			{
				Node* toInsert = &clients[nodesToInsert[bestNodeIdx].clientIdx];
				toInsert->prev = bestPred;
				toInsert->next = bestPred->next;
				bestPred->next->prev = toInsert;
				bestPred->next = toInsert;
				updateRouteData(&routes[r]);
				insertedNode = true;
				unassignedNodeIndices.erase(bestNodeIdx);
			}
		}
	}
	// Insert all unassigned nodes at the back of the last route. We assume that typically there
	// are no unassigned nodes left, because there are plenty routes, but we have to make sure that
	// all nodes are assigned.
	if (unassignedNodeIndices.size() > 0)
	{
		int lastRouteIdx = routes.size() - 1;
		Node* prevNode = depotsEnd[lastRouteIdx].prev; // Last node before finish depot in last route.

		while (unassignedNodeIndices.size() > 0)
		{
			int idx = *unassignedNodeIndices.begin();
			Node* toInsert = &clients[nodesToInsert[idx].clientIdx];
			toInsert->prev = prevNode;
			toInsert->next = prevNode->next;
			prevNode->next->prev = toInsert;
			prevNode->next = toInsert;
			unassignedNodeIndices.erase(idx);
		}

		updateRouteData(&routes[lastRouteIdx]);
	}

	// Register the solution produced by the construction heuristic in the individual.
	exportIndividual(indiv);
}

void LocalSearch::run(Individual* indiv, double penaltyVehicleCapacityLS, double penaltyTimeWarpLS, double penaltyParcelPointCapacityLS, bool applyParcelPointNeighborhood)
{
	static const bool neverIntensify = params->config.intensificationProbabilityLS == 0;
	static const bool alwaysIntensify = params->config.intensificationProbabilityLS == 100;
	const bool runLS_INT = params->rng() % 100 < (unsigned int) params->config.intensificationProbabilityLS;

	this->penaltyVehicleCapacityLS = penaltyVehicleCapacityLS;
	this->penaltyParcelPointCapacityLS = penaltyParcelPointCapacityLS;
	this->penaltyTimeWarpLS = penaltyTimeWarpLS;
	loadIndividual(indiv);
	// if(shuffleClientOrderInPP){
	// 	std::cout<<"Helloo"<<std::endl;
	// }

	// Shuffling the order of the nodes explored by the LS to allow for more diversity in the search
	std::shuffle(orderNodes.begin(), orderNodes.end(), params->rng);
	std::shuffle(orderRoutes.begin(), orderRoutes.end(), params->rng);
	for (int i = 1; i <= params->nbClients; i++)
		if (params->rng() % params->config.nbGranular == 0)  // Designed to use O(nbGranular x n) time overall to avoid possible bottlenecks
			std::shuffle(params->correlatedVertices[i].begin(), params->correlatedVertices[i].end(), params->rng);


	std::vector< std::vector < Node* >> correlatedParcelPointPointers(params->nbClients);
	if (applyParcelPointNeighborhood)
	{
		for (int posU = 0; posU < params->nbClients; posU++)
		{
			nodeU = &clients[orderNodes[posU]];
			for (auto& c : parcelPointsPointer)
			{
				if(params->timeCost.get(nodeU->cour, c->cour) <= params->maxWalking)
				{
					correlatedParcelPointPointers[posU].push_back(c);
				}
			}
		}
	}

	searchCompleted = false;
	for (loopID = 0; !searchCompleted; loopID++)
	{
		if (loopID > 1)
		{
			// Allows at least two loops since some moves involving empty routes are not checked at the first loop
			searchCompleted = true;
		}
		if (applyParcelPointNeighborhood)
		{
			for (int posU = 0; posU < params->nbClients; posU++)
			{
				nodeU = &clients[orderNodes[posU]];
				setLocalVariablesRouteUForParcelPoints();
				// MOVES INVOLVING MOVING MULTIPLE CUSTOMERS TO PARCEL POINTS
				if (nodeU->cour == nodeU->servedBy)
				{
					for (auto parcelPoint : correlatedParcelPointPointers[posU])
					{
						if (MultipleClientsToParcelPoint(parcelPoint, indiv)) break; // Move Customer to Parcel Point
					}
				}

				// With 50% apply parcel to client procedure and with 50% the from parcel point to parcel point procedure
				if (params->rng() % 100 < (unsigned int) 80){
					// MOVES INVOLVING MOVING MULTIPLE CUSTOMERS FROM PARCEL POINTS TO CUSTOMER
					setLocalVariablesRouteUForParcelPoints();
					if (nodeU->cour != nodeU->servedBy)
					{
						if (MultipleClientsBackToClients(nodeU->servedByParcelPoint, indiv)) break; // Move Customer to Parcel Point
					}
				} else{
					// MOVES INVOLVING MOVING MULTIPLE CUSTOMERS FROM ONE PARCEL POINTS TO ANOTHER PARCEL POINT
					if (nodeU->cour != nodeU->servedBy)
					{
						for (auto toParcelPoint : correlatedParcelPointPointers[posU])
						{
							if (toParcelPoint->cour != nodeU->servedBy) // We do not perform the neighborhood when the customer is already served by that parcel point
							{
								setLocalVariablesRouteUForParcelPoints();
								if (MultipleClientsFromParcelPointToParcelPoint(nodeU->servedByParcelPoint, toParcelPoint, indiv)){
									posU = params->nbClients;
									break;// Move Customer to Parcel Point
								}
							}
						}
					}
				}
			}
		}
		/* CLASSICAL ROUTE IMPROVEMENT (RI) MOVES SUBJECT TO A PROXIMITY RESTRICTION */
		for (int posU = 0; posU < params->nbClients; posU++)
		{
			nodeU = &clients[orderNodes[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;

			const auto& correlated = params->correlatedVertices[nodeU->cour];
			for (const auto& v : correlated)
			{
				nodeV = &clients[v];
				if (nodeU->servedBy != nodeV->servedBy && nodeU->servedBy != nodeU->next->servedBy && nodeV->servedBy != nodeV->next->servedBy)
				{
					if (loopID == 0 || std::max(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
					{
						// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
						setLocalVariablesRouteU();
						setLocalVariablesRouteV();
						if (MoveSingleClient()) continue; // RELOCATE
						if (MoveTwoClients()) continue; // RELOCATE
						if (MoveTwoClientsReversed()) continue; // RELOCATE
						if (nodeUIndex < nodeVIndex && SwapTwoSingleClients()) continue; // SWAP
						if (SwapTwoClientsForOne()) continue; // SWAP
						if (nodeUIndex < nodeVIndex && SwapTwoClientPairs()) continue; // SWAP
						if (routeU->cour < routeV->cour && TwoOptBetweenTrips()) continue; // 2-OPT*
						if (routeU == routeV && TwoOptWithinTrip()) continue; // 2-OPT
						// Trying moves that insert nodeU directly after the depot
						if (nodeV->prev->isDepot)
						{
							nodeV = nodeV->prev;
							setLocalVariablesRouteV();
							if (MoveSingleClient()) continue; // RELOCATE
							if (MoveTwoClients()) continue; // RELOCATE
							if (MoveTwoClientsReversed()) continue; // RELOCATE
							if (routeU->cour < routeV->cour && TwoOptBetweenTrips()) continue; // 2-OPT*
						}
					}
				}
			}
			/* MOVES INVOLVING AN EMPTY ROUTE -- NOT TESTED IN THE FIRST LOOP TO AVOID INCREASING TOO MUCH THE FLEET SIZE */
			if (loopID > 0 && !emptyRoutes.empty())
			{
				nodeV = routes[*emptyRoutes.begin()].depot;
				setLocalVariablesRouteU();
				setLocalVariablesRouteV();
				if (MoveSingleClient()) continue; // RELOCATE
				if (MoveTwoClients()) continue; // RELOCATE
				if (MoveTwoClientsReversed()) continue; // RELOCATE
				if (TwoOptBetweenTrips()) continue; // 2-OPT*
			}
		}
		/* (SWAP*) MOVES LIMITED TO ROUTE PAIRS WHOSE CIRCLE SECTORS OVERLAP */
		if (!neverIntensify && searchCompleted && (alwaysIntensify || runLS_INT))
		{
			for (int rU = 0; rU < params->nbVehicles; rU++)
			{
				routeU = &routes[orderRoutes[rU]];
				if (routeU->nbCustomers == 0)
				{
					continue;
				}

				int lastTestLargeNbRouteU = routeU->whenLastTestedLargeNb;
				routeU->whenLastTestedLargeNb = nbMoves;
				for (int rV = 0; rV < params->nbVehicles; rV++)
				{
					routeV = &routes[orderRoutes[rV]];
					if (routeV->nbCustomers == 0 || routeU->cour >= routeV->cour)
					{
						continue;
					}

					if (loopID > 0 && std::max(routeU->whenLastModified, routeV->whenLastModified) <= lastTestLargeNbRouteU)
					{
						continue;
					}

					if (!CircleSector::overlap(routeU->sector, routeV->sector, params->circleSectorOverlapTolerance))
					{
						continue;
					}

					if (!RelocateStar())
					{
						if(params->config.skipSwapStarDist || !swapStar(false)){
							if (params->config.useSwapStarTW)
							{
								swapStar(true);
							}
						}
					}
				}
			}
		}
	}
	correlatedParcelPointPointers.clear();
	// Register the solution produced by the LS in the individual
	exportIndividual(indiv);

}

void LocalSearch::RemoveCustomerFromServedClientsVector(std::vector<Node*> & V, Node* customerToDelete) {
	V.erase(
        std::remove_if(V.begin(), V.end(), [&](Node* const & o) {
            return o->cour == customerToDelete->cour;
        }),
        V.end());

}

void LocalSearch::shuffleCustomersInParcelPoints(){
	for (int r = 0; r < params->nbVehicles; r++)
	{
		Route* myRoute = &routes[r];
		for(Node* pp: parcelPointsPointer){
			if (pp->servedClients[myRoute->cour].size() > 1){
				prevNodePP = pp->servedClients[myRoute->cour][0]->prev;
				while (pp->cour== prevNodePP->servedBy)
				{
					prevNodePP = prevNodePP->prev;
				}

				nextNodePP = pp->servedClients[myRoute->cour][pp->servedClients[myRoute->cour].size()-1];
				while (pp->cour == nextNodePP->servedBy)
				{
					nextNodePP = nextNodePP->next;
				}

				std::shuffle(pp->servedClients[myRoute->cour].begin(), pp->servedClients[myRoute->cour].end(), params->rng);
				for (int customerIdx = 0; (size_t) customerIdx < pp->servedClients[myRoute->cour].size(); ++customerIdx) {
					if (customerIdx == 0){
						pp->servedClients[myRoute->cour][customerIdx]->prev = prevNodePP;
						prevNodePP->next = pp->servedClients[myRoute->cour][customerIdx]; 
					}else{
						pp->servedClients[myRoute->cour][customerIdx]->prev = pp->servedClients[myRoute->cour][customerIdx-1];
					}
					pp->position = prevNodePP->position + customerIdx + 1;

					if((size_t) customerIdx == pp->servedClients[myRoute->cour].size()-1){
						pp->servedClients[myRoute->cour][customerIdx]->next = nextNodePP;
						nextNodePP->prev = pp->servedClients[myRoute->cour][customerIdx];
					}else{
						pp->servedClients[myRoute->cour][customerIdx]->next = pp->servedClients[myRoute->cour][customerIdx+1];
					}
				}
				
			}
		}
		updateRouteData(myRoute);
	}
}

void LocalSearch::setLocalVariablesRouteU()
{
	routeU = nodeU->route;
	nodeX = nodeU->next;
	nodeXReal = nodeU->next;
	nodeXTW = nodeU->next;
	loadU = params->cli[nodeU->cour].demand;
	nodeUPrev = nodeU->prev;
	if (nodeU->servedBy == nodeUPrev->servedBy){
		while (nodeU->servedBy == nodeUPrev->servedBy)
		{
			loadU += params->cli[nodeUPrev->cour].demand;
			nodeUPrev = nodeUPrev->prev;
		}
	}


	loadX = params->cli[nodeX->cour].demand;
	if (!nodeX->isDepot){
		while (nodeX->servedBy == nodeX->next->servedBy){
			nodeX = nodeX->next;
			loadX += params->cli[nodeX->cour].demand;
		}
	}

	if (!nodeXTW->isDepot){
		while (nodeXTW->servedBy == nodeU->servedBy){
			nodeXTW = nodeXTW->next;
		}
	}

	nodeUPrevTW = nodeUPrev->next;
	nodeXTW = nodeX->prev;

	nodeXNext = nodeX->next;
	nodeUIndex = nodeU->servedBy;
	nodeUPrevIndex = nodeUPrev->servedBy;
	nodeXIndex = nodeX->servedBy;
	nodeXNextIndex = nodeXNext->servedBy;
	serviceU = params->cli[nodeU->cour].serviceDuration;
	serviceX = params->cli[nodeX->cour].serviceDuration;
	routeUTimeWarp = routeU->twData.timeWarp > 0;
	routeULoadPenalty = routeU->load > params->vehicleCapacity;
}

void LocalSearch::setLocalVariablesRouteUForParcelPoints()
{
	routeU = nodeU->route;
	nodeX = nodeU->next;
	nodeXReal = nodeU->next;
	nodeXTW = nodeU->next;
	loadU = params->cli[nodeU->cour].demand;
	nodeUPrev = nodeU->prev;
	while (nodeU->servedBy == nodeUPrev->servedBy)
	{
		loadU += params->cli[nodeUPrev->cour].demand;
		nodeUPrev = nodeUPrev->prev;
	}


	loadX = params->cli[nodeX->cour].demand;
	while (nodeU->servedBy == nodeX->servedBy){
		nodeX = nodeX->next;
		loadX += params->cli[nodeX->cour].demand;
	}


	// printRoute(routeU, false);
	// printRoute(routeU, true);
	// std::cout<<" "<<nodeUPrev->servedBy<<" "<<nodeU->servedBy<<" "<<nodeX->servedBy<<std::endl;


	if (!nodeXTW->isDepot){
		while (nodeXTW->servedBy == nodeU->servedBy){
			nodeXTW = nodeXTW->next;
		}
	}

	nodeUPrevTW = nodeUPrev->next;
	nodeXTW = nodeX->prev;

	nodeXNext = nodeX->next;
	nodeUIndex = nodeU->servedBy;
	nodeUPrevIndex = nodeUPrev->servedBy;
	nodeXIndex = nodeX->servedBy;
	nodeXNextIndex = nodeXNext->servedBy;
	serviceU = params->cli[nodeU->cour].serviceDuration;
	serviceX = params->cli[nodeX->cour].serviceDuration;
	routeUTimeWarp = routeU->twData.timeWarp > 0;
	routeULoadPenalty = routeU->load > params->vehicleCapacity;
}


void LocalSearch::setLocalVariablesRouteV()
{
	routeV = nodeV->route;
	nodeY = nodeV->next;
	nodeYReal = nodeV->next;
	nodeVPrev = nodeV->prev;
	loadV = params->cli[nodeV->cour].demand;
	if (!nodeV->isDepot){
		if (nodeV->servedBy == nodeVPrev->servedBy){
			while (nodeV->servedBy == nodeVPrev->servedBy)
			{
				loadV += params->cli[nodeVPrev->cour].demand;
				nodeVPrev = nodeVPrev->prev;
			}
		}
	}

	loadY = params->cli[nodeY->cour].demand;
	if (!nodeY->isDepot){
		while (nodeY->servedBy == nodeY->next->servedBy){
			nodeY = nodeY->next;
			loadY += params->cli[nodeY->cour].demand;
		}
	}

	nodeVPrevTW = nodeVPrev->next;
	nodeYTW = nodeY->prev;

	nodeYNext = nodeY->next;
	nodeYNextIndex = nodeYNext->servedBy;
	nodeVIndex = nodeV->servedBy;
	nodeVPrevIndex = nodeVPrev->servedBy;
	nodeYIndex = nodeY->servedBy;
	serviceV = params->cli[nodeV->servedBy].serviceDuration;
	serviceY = params->cli[nodeY->cour].serviceDuration;
	routeVTimeWarp = routeV->twData.timeWarp > 0;
	routeVLoadPenalty = routeV->load > params->vehicleCapacity;
}

bool::LocalSearch::SwapClientsInParcelPoints(Individual* indiv)
{
	if (params->timeCost.get(nodeU->cour, nodeV->servedBy) - params->maxWalking >  MY_EPSILON) return false;
	if (params->timeCost.get(nodeV->cour, nodeU->servedBy) - params->maxWalking >  MY_EPSILON) return false;
	if (nodeU == nodeV->prev || nodeU == nodeV->next || nodeU->prev == nodeV) return false;

	double distanceCosts = params->priceSensitivity - nodeU->discount + params->priceSensitivity - nodeV->discount;
	double costDifference = distanceCosts;
	if(nodeU->route != nodeV->route){
		costDifference += penaltyExcessLoadVehicle(nodeU->route->load + params->cli[nodeV->cour].demand - params->cli[nodeU->cour].demand) + penaltyExcessLoadVehicle(nodeV->route->load + params->cli[nodeU->cour].demand - params->cli[nodeV->cour].demand) - penaltyExcessLoadVehicle(nodeU->route->load) - penaltyExcessLoadVehicle(nodeV->route->load);
	}

	if (costDifference > -MY_EPSILON) return false;
	swapParcelPointClients(nodeU, nodeV, indiv);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(nodeU->route);
	if (nodeU->route != nodeV->route) updateRouteData(nodeV->route);

	return true;
}


bool::LocalSearch::MultipleClientsFromParcelPointToParcelPoint(Node* fromParcelPoint, Node* toParcelPoint, Individual* I)
{
	if (params->timeCost.get(nodeU->cour, toParcelPoint->cour) - params->maxWalking >  MY_EPSILON) return false;
	double distanceDifference = 0;
	int beforeOrAfter = 0;
	TimeWindowData routeUTwData;
	if (fromParcelPoint->servedClients[nodeU->route->cour].size() > 1 && toParcelPoint->servedClients[nodeU->route->cour].size() > 0){
		distanceDifference = 0;
		routeUTwData = routeU->twData;
	} else if (toParcelPoint->servedClients[nodeU->route->cour].size() < 1 && fromParcelPoint->servedClients[nodeU->route->cour].size() > 1){
			double distanceBefore = params->timeCost.get(nodeUPrevIndex, toParcelPoint->cour) + params->timeCost.get(toParcelPoint->cour, nodeU->servedBy);
			TimeWindowData routeUTwData1 = MergeTWDataRecursiveNewIndices(nodeUPrev->prefixTwData, nodeU->twData, nodeUPrev->servedBy, toParcelPoint->cour);
			routeUTwData1 = MergeTWDataRecursiveNewIndices(routeUTwData1, nodeU->postfixTwData, toParcelPoint->cour, nodeU->servedBy);
			double twCostsBefore = penaltyTimeWindows(routeUTwData1);
			
			double distanceAfter = params->timeCost.get(nodeU->servedBy, toParcelPoint->cour) + params->timeCost.get(toParcelPoint->cour, nodeXIndex);
			TimeWindowData routeUTwData2 = MergeTWDataRecursiveNewIndices(nodeU->prefixTwData, nodeU->twData, fromParcelPoint->cour, toParcelPoint->cour); 
			routeUTwData2 = MergeTWDataRecursiveNewIndices(routeUTwData2, nodeX->postfixTwData, toParcelPoint->cour, nodeX->servedBy); 
			double twCostsAfter = penaltyTimeWindows(routeUTwData2);

		if (distanceBefore + twCostsBefore < distanceAfter + twCostsAfter){
			distanceDifference = distanceBefore;
			routeUTwData = routeUTwData1;
			beforeOrAfter = 1; // customer is located before other customers served by the parcel point.

		}else{
			distanceDifference = distanceAfter;
			routeUTwData = routeUTwData2;
			beforeOrAfter = 2; // customer is located after other customers served by the parcel point.
		}

	} else if (toParcelPoint->servedClients[nodeU->route->cour].size() < 1 && fromParcelPoint->servedClients[nodeU->route->cour].size() < 2){
		beforeOrAfter = 3;
		distanceDifference = params->timeCost.get(nodeUPrevIndex, toParcelPoint->cour) + params->timeCost.get(toParcelPoint->cour, nodeXIndex) - params->timeCost.get(nodeUPrevIndex, nodeUIndex) - params->timeCost.get(nodeUIndex, nodeXIndex);
		routeUTwData = MergeTWDataRecursiveNewIndices(nodeU->prefixTwData, nodeU->twData, nodeU->prev->servedBy, toParcelPoint->cour);
		routeUTwData = MergeTWDataRecursive(routeUTwData, nodeU->next->postfixTwData);
	} else{
		if (toParcelPoint->servedClients[nodeU->route->cour][0]->position < fromParcelPoint->servedClients[nodeU->route->cour][0]->position){
			beforeOrAfter = 4;
			routeUTwData = MergeTWDataRecursive(toParcelPoint->servedClients[nodeU->route->cour][0]->prefixTwData, getRouteSegmentTwData(toParcelPoint->servedClients[nodeU->route->cour][toParcelPoint->servedClients[nodeU->route->cour].size()-1], nodeU->prev));
			routeUTwData = MergeTWDataRecursive(routeUTwData, nodeU->next->postfixTwData);
		}else{
			beforeOrAfter = 5;
			routeUTwData = MergeTWDataRecursive(nodeU->prev->prefixTwData, getRouteSegmentTwData(nodeU->next, routeU->depot));
		}

		distanceDifference = -params->timeCost.get(nodeUPrevIndex, nodeUIndex) - params->timeCost.get(nodeUIndex, nodeXIndex) + params->timeCost.get(nodeUPrevIndex, nodeXIndex);
	}

	double penaltyDifference =  penaltyExcessLoadParcelPoint(toParcelPoint->totalClientsServed + 1, params->cellsPerParcelPoint[toParcelPoint->cour-params->nbClients-1]) + penaltyExcessLoadParcelPoint(fromParcelPoint->totalClientsServed - 1, params->cellsPerParcelPoint[fromParcelPoint->cour-params->nbClients-1]) - penaltyExcessLoadParcelPoint(fromParcelPoint->totalClientsServed, params->cellsPerParcelPoint[fromParcelPoint->cour-params->nbClients-1]) - penaltyExcessLoadParcelPoint(toParcelPoint->totalClientsServed,params->cellsPerParcelPoint[toParcelPoint->cour-params->nbClients-1]);
	double timeWindowCosts = penaltyTimeWindows(routeUTwData);
	double additionalCosts = distanceDifference + penaltyDifference + timeWindowCosts - routeU->penalty + penaltyExcessLoadVehicle(routeU->load);
	if (additionalCosts < -MY_EPSILON)
	{
		std::vector<Node*> moveList;
		moveList.push_back(nodeU);
		moveMultipleClientsFromParcelPointToParcelPoint(moveList, fromParcelPoint, toParcelPoint, I, beforeOrAfter);
		nbMoves++; // Increment move counter before updating route data
		searchCompleted = false;
		updateRouteData(routeU);
		return true;
	} else{
		int counter = 2;
		bool costBenefits = false;
		int additionalLoad = params->cli[nodeU->cour].demand;
		Node* currentNode = nodeU;
		std::vector<Node*> moveList;
		moveList.push_back(currentNode);
		while (!costBenefits)
		{
			Node* nextNode = currentNode->next;
			if (nextNode->isDepot) break; // Leave loop when next node is the depot
			if (nextNode->servedBy != currentNode->servedBy) break; // Leave loop when next node is served by someone else
			if (params->timeCost.get(nextNode->cour, toParcelPoint->cour) - params->maxWalking >  MY_EPSILON) break; // Leave loop when next node is too far away from the parcel point
			additionalLoad += params->cli[nextNode->cour].demand; // Increment load


			if (fromParcelPoint->servedClients[nodeU->route->cour].size() == (size_t)counter){
				distanceDifference += params->timeCost.get(nodeUPrevIndex, fromParcelPoint->cour) + params->timeCost.get(fromParcelPoint->cour, nextNode->next->servedBy);
				if (toParcelPoint->servedClients[nodeU->route->cour].size()>0){
					if (toParcelPoint->servedClients[nodeU->route->cour][0]->position < fromParcelPoint->servedClients[nodeU->route->cour][0]->position){
						beforeOrAfter = 6;
						routeUTwData = MergeTWDataRecursive(toParcelPoint->servedClients[nodeU->route->cour][0]->prefixTwData, getRouteSegmentTwData(toParcelPoint->servedClients[nodeU->route->cour][toParcelPoint->servedClients[nodeU->route->cour].size()-1], routeU->depot));
					}else{
						beforeOrAfter = 7;
						routeUTwData = MergeTWDataRecursive(nodeU->prev->prefixTwData, getRouteSegmentTwData(nodeX, routeU->depot));
					}
				}else{
					beforeOrAfter = 8;
				}
			}


			penaltyDifference =  penaltyExcessLoadParcelPoint(toParcelPoint->totalClientsServed + counter, params->cellsPerParcelPoint[toParcelPoint->cour-params->nbClients-1]) + penaltyExcessLoadParcelPoint(fromParcelPoint->totalClientsServed - counter, params->cellsPerParcelPoint[fromParcelPoint->cour-params->nbClients-1]) - penaltyExcessLoadParcelPoint(fromParcelPoint->totalClientsServed,params->cellsPerParcelPoint[fromParcelPoint->cour-params->nbClients-1]) - penaltyExcessLoadParcelPoint(toParcelPoint->totalClientsServed, params->cellsPerParcelPoint[toParcelPoint->cour-params->nbClients-1]);

			additionalCosts = distanceDifference + penaltyDifference + timeWindowCosts - routeU->penalty + penaltyExcessLoadVehicle(routeU->load);
			//std::cout<<toParcelPoint->cour<<" "<<additionalCosts<<std::endl;
			if (additionalCosts < -MY_EPSILON){
				moveList.push_back(nextNode);
				costBenefits = true;
			} else{
				moveList.push_back(nextNode);
				counter += 1;
				currentNode = nextNode;
			}
		}

		// Return false when there are no cost benefits
		if (!costBenefits) return false;
		moveMultipleClientsFromParcelPointToParcelPoint(moveList, fromParcelPoint, toParcelPoint, I, beforeOrAfter);
		nbMoves++; // Increment move counter before updating route data
		searchCompleted = true;
		updateRouteData(routeU);
		return true;
	}
}


bool LocalSearch::MultipleClientsToParcelPoint(Node* parcelPoint, Individual* I)
{
	if (params->timeCost.get(nodeU->cour, parcelPoint->cour) - params->maxWalking >  MY_EPSILON) return false;
	double distanceCosts = 0;
	int checkMark = 0;
	TimeWindowData routeUTwData;
	if (parcelPoint->servedClients[nodeU->route->cour].size() < 1){
		distanceCosts = params->timeCost.get(nodeUPrevIndex, parcelPoint->cour) + params->timeCost.get(parcelPoint->cour, nodeXIndex) - params->timeCost.get(nodeUPrevIndex, nodeUIndex) - params->timeCost.get(nodeUIndex, nodeXIndex);
		routeUTwData = MergeTWDataRecursiveNewIndicesNewTWNewParcelPoint(nodeUPrev->prefixTwData, nodeU->twData, nodeUPrev->servedBy, parcelPoint->cour, nodeUPrev->prefixTwData.earliestArrival,nodeUPrev->prefixTwData.latestArrival,params->cli[0].earliestArrival,params->cli[0].latestArrival);
		routeUTwData = MergeTWDataRecursiveNewIndicesNewTW(routeUTwData, nodeU->next->postfixTwData, parcelPoint->cour, nodeU->next->servedBy, routeUTwData.earliestArrival, routeUTwData.latestArrival, nodeU->next->postfixTwData.earliestArrival, nodeU->next->postfixTwData.latestArrival);

	} else{
		checkMark = 1;
		distanceCosts = params->timeCost.get(nodeUPrevIndex, nodeXIndex)-params->timeCost.get(nodeUPrevIndex, nodeU->servedBy)-params->timeCost.get(nodeU->servedBy, nodeXIndex);
		routeUTwData = MergeTWDataRecursive(nodeUPrev->prefixTwData, nodeU->next->postfixTwData);
	}

	double penaltyCosts = penaltyExcessLoadParcelPoint(parcelPoint->totalClientsServed + 1, params->cellsPerParcelPoint[parcelPoint->cour-params->nbClients-1]) - penaltyExcessLoadParcelPoint(parcelPoint->totalClientsServed, params->cellsPerParcelPoint[parcelPoint->cour-params->nbClients-1]);
	double discountCosts = params->priceSensitivity;
	double timeWindowCosts = penaltyTimeWindows(routeUTwData);

	double additionalCosts = distanceCosts + penaltyCosts + discountCosts + timeWindowCosts- routeU->penalty + penaltyExcessLoadVehicle(routeU->load);
	// If single client move already leads to improvements, we only move that one customer. Else, we try to move multiple customers until we reached an improvement. If no improvement is reached, we leave the function.
	if (additionalCosts < -MY_EPSILON){
		std::vector<Node*> moveList;
		moveList.push_back(nodeU);
		moveMultipleClientsToParcelPoint(moveList, parcelPoint, I);
		nbMoves++; // Increment move counter before updating route data
		searchCompleted = true;
		updateRouteData(routeU);
		return true;
	} else {
		int counter = 2;
		bool costBenefits = false;
		int additionalLoad = params->cli[nodeU->cour].demand;
		Node* currentNode = nodeU;
		std::vector<Node*> moveList;
		moveList.push_back(currentNode);
		while (!costBenefits)
		{
			Node* nextNode = currentNode->next;
			if (nextNode->isDepot) return false; // Leave loop when next node is the depot
			if (params->timeCost.get(nextNode->cour, parcelPoint->cour) - params->maxWalking >  MY_EPSILON) return false;
			if (nextNode->servedBy != nextNode->cour) return false; // Leave loop when next node is already served by another parcel point
			additionalLoad += params->cli[nextNode->cour].demand; // Increment load

			// Calculate additional costs
			if (checkMark == 1){
				distanceCosts = distanceCosts + params->timeCost.get(nodeUPrevIndex, nextNode->next->servedBy) - params->timeCost.get(nodeUPrevIndex, nextNode->cour) - params->timeCost.get(nextNode->cour, nextNode->next->servedBy);
				routeUTwData = MergeTWDataRecursive(nodeUPrev->prefixTwData, nextNode->next->postfixTwData);
			} else{
				distanceCosts = distanceCosts + params->timeCost.get(parcelPoint->cour, nextNode->next->servedBy) - params->timeCost.get(parcelPoint->cour, nextNode->cour) - params->timeCost.get(nextNode->cour, nextNode->next->servedBy);
				routeUTwData = MergeTWDataRecursiveNewIndicesNewTW(nodeUPrev->prefixTwData, nodeU->twData, nodeUPrev->servedBy, parcelPoint->cour, params->cli[nodeUPrev->servedBy].earliestArrival,params->cli[nodeUPrev->servedBy].latestArrival,params->cli[0].earliestArrival,params->cli[0].latestArrival);
				routeUTwData = MergeTWDataRecursiveNewIndicesNewTW(routeUTwData, nextNode->next->postfixTwData, parcelPoint->cour, nodeU->next->servedBy, params->cli[0].earliestArrival,params->cli[0].latestArrival,params->cli[nextNode->next->servedBy].earliestArrival,params->cli[nextNode->next->servedBy].latestArrival);
			}

			penaltyCosts = penaltyExcessLoadParcelPoint(parcelPoint->totalClientsServed + counter, params->cellsPerParcelPoint[parcelPoint->cour-params->nbClients-1]) - penaltyExcessLoadParcelPoint(parcelPoint->totalClientsServed,params->cellsPerParcelPoint[parcelPoint->cour-params->nbClients-1]);
			discountCosts += params->priceSensitivity;
			routeUTwData = MergeTWDataRecursiveNewIndicesNewTW(nodeUPrev->prefixTwData, nodeU->twData, nodeUPrev->servedBy, parcelPoint->cour, params->cli[nodeUPrev->servedBy].earliestArrival,params->cli[nodeUPrev->servedBy].latestArrival,params->cli[0].earliestArrival,params->cli[0].latestArrival);
			routeUTwData = MergeTWDataRecursiveNewIndicesNewTW(routeUTwData, nextNode->next->postfixTwData, parcelPoint->cour, nextNode->next->servedBy, params->cli[0].earliestArrival,params->cli[0].latestArrival,params->cli[nextNode->next->servedBy].earliestArrival,params->cli[nextNode->next->servedBy].latestArrival);
			timeWindowCosts = penaltyTimeWindows(routeUTwData) - routeU->penalty + penaltyExcessLoadVehicle(routeU->load);

			additionalCosts = distanceCosts + penaltyCosts + discountCosts + timeWindowCosts;
			if (additionalCosts < -MY_EPSILON){
				moveList.push_back(nextNode);
				costBenefits = true;
			} else{
				moveList.push_back(nextNode);
				counter += 1;
				currentNode = nextNode;
			}
		}
		// Return false when there are no cost benefits
		if (!costBenefits) return false;
		moveMultipleClientsToParcelPoint(moveList, parcelPoint, I);
		nbMoves++; // Increment move counter before updating route data
		searchCompleted = false;
		updateRouteData(routeU);
		return true;
	}
}

bool LocalSearch::MultipleClientsBackToClients(Node* parcelPoint, Individual* I)
{
	double distanceCosts = 0;
	int beforeOrAfter = 0;
	TimeWindowData routeUTwData, routeUTwDataBefore, routeUTwDataAfter, routeUTwData1, routeUTwData2;

	if (parcelPoint->servedClients[routeU->cour].size() > 1){
		if (nodeUPrev->next->servedBy != nodeU->servedBy && nodeU->servedBy != nodeX->prev->servedBy){
			return false; // Return false if parcel point serves multiple customers but they are not served consecutively --> First fix route.
		} else{
			double distanceCostsBefore = params->timeCost.get(nodeUPrevIndex, nodeU->cour) + params->timeCost.get(nodeU->cour, nodeU->servedBy) - params->timeCost.get(nodeUPrevIndex, nodeU->servedBy);
			routeUTwData1 = MergeTWDataRecursiveNewIndicesNewTW(nodeUPrev->prefixTwData, nodeU->twData, nodeUPrev->servedBy, nodeU->cour, nodeUPrev->prefixTwData.earliestArrival, nodeUPrev->prefixTwData.latestArrival,params->cli[nodeU->cour].earliestArrival,params->cli[nodeU->cour].latestArrival);
			routeUTwDataBefore = MergeTWDataRecursiveNewIndicesNewTWNoParcelPoint(routeUTwData1, nodeU->postfixTwData, nodeU->cour, nodeU->servedBy, routeUTwData1.earliestArrival, routeUTwData1.latestArrival, nodeU->postfixTwData.earliestArrival, nodeU->postfixTwData.latestArrival);
			double twCostsBefore = penaltyTimeWindows(routeUTwDataBefore);

			double distanceCostsAfter = params->timeCost.get(nodeU->servedBy, nodeU->cour) + params->timeCost.get(nodeU->cour, nodeXIndex) - params->timeCost.get(nodeU->servedBy, nodeXIndex);
			routeUTwData2 = MergeTWDataRecursiveNewIndicesNewTW(nodeU->prefixTwData, nodeU->twData, nodeU->servedBy, nodeU->cour, nodeU->prefixTwData.earliestArrival,nodeU->prefixTwData.latestArrival,params->cli[nodeU->cour].earliestArrival,params->cli[nodeU->cour].latestArrival);
			routeUTwDataAfter = MergeTWDataRecursiveNewIndicesNewTWNoParcelPoint(routeUTwData2, nodeX->postfixTwData, nodeU->cour, nodeX->servedBy, routeUTwData2.earliestArrival,routeUTwData2.latestArrival,nodeX->postfixTwData.earliestArrival,nodeX->postfixTwData.latestArrival);
			double twCostsAfter = penaltyTimeWindows(routeUTwDataBefore);

			if (distanceCostsBefore + twCostsBefore < distanceCostsAfter + twCostsAfter){
				distanceCosts = distanceCostsBefore;
				routeUTwData = routeUTwDataBefore;
				beforeOrAfter = 1; // customer is located before other customers served by the parcel point.
				
			}else{
				distanceCosts = distanceCostsAfter;
				routeUTwData = routeUTwDataAfter;
				beforeOrAfter = 2; // customer is located after other customers served by the parcel point.

			}
		}
	} else{
		distanceCosts = params->timeCost.get(nodeU->prev->servedBy, nodeU->cour) + params->timeCost.get(nodeU->cour, nodeU->next->servedBy) - params->timeCost.get(nodeU->prev->servedBy, nodeU->servedBy) - params->timeCost.get(nodeU->servedBy, nodeU->next->servedBy);
		routeUTwData = MergeTWDataRecursiveNewIndicesNewTW(nodeU->prev->prefixTwData, nodeU->twData, nodeU->prev->servedBy, nodeU->cour, nodeU->prev->prefixTwData.earliestArrival,nodeU->prev->prefixTwData.latestArrival,params->cli[nodeU->cour].earliestArrival,params->cli[nodeU->cour].latestArrival);
		routeUTwData = MergeTWDataRecursiveNewIndicesNewTW(routeUTwData, nodeU->next->postfixTwData, nodeU->cour, nodeU->next->servedBy, routeUTwData.earliestArrival,routeUTwData.latestArrival,nodeU->next->postfixTwData.earliestArrival,nodeU->next->postfixTwData.latestArrival);
	}
	
	
	double timeWindowCosts = penaltyTimeWindows(routeUTwData);
	double discountCosts = -nodeU->discount;
	double penaltyCosts = penaltyExcessLoadParcelPoint(parcelPoint->totalClientsServed- 1, params->cellsPerParcelPoint[parcelPoint->cour-params->nbClients-1]) - penaltyExcessLoadParcelPoint(parcelPoint->totalClientsServed, params->cellsPerParcelPoint[parcelPoint->cour-params->nbClients-1]);
	
	double additionalCosts = distanceCosts + discountCosts + penaltyCosts + timeWindowCosts - penaltyTimeWindows(routeU->twData);
	// If single client move already leads to improvements, we only move that one customer. Else, we try to move multiple customers until we reached an improvement. If no improvement is reached, we leave the function.
	if (additionalCosts < -MY_EPSILON){
		double oldCost = getCostSol().penalizedCost;
		double costsBefore = getCostSol().timeWarp;
		std::vector<Node*> moveList;
		moveList.push_back(nodeU);
		moveMultipleClientsBackToClients(moveList, parcelPoint, I, beforeOrAfter);
		nbMoves++; // Increment move counter before updating route data
		searchCompleted = false;
		updateRouteData(routeU);
		if (getCostSol().penalizedCost > oldCost){
			printRoute(routeU,true);
			printRoute(routeU,false);
			std::cout<<beforeOrAfter<<" "<<nodeU->cour<<" "<<" "<<penaltyTimeWarpLS<<" "<<" Hello"<<std::endl;
			std::cout<<getCostSol().timeWarp*penaltyTimeWarpLS<<" "<<costsBefore*penaltyTimeWarpLS<<" "<<timeWindowCosts<<std::endl;
		}
		return true;
	} else {
		int counter = 2;
		bool costBenefits = false;
		Node* currentNode = nodeU;
		Node* nextNode = currentNode;
		double allDemand = params->cli[nodeU->cour].demand;
		std::vector<Node*> moveList;
		moveList.push_back(currentNode);
		while (!costBenefits)
		{
			nextNode = currentNode->next;
			if (nextNode->isDepot) return false; // Leave loop when next node is the depot
			if (nextNode->servedBy != currentNode->servedBy) return false; // Leave loop when next node is served by another parcel point
			allDemand += params->cli[nextNode->cour].demand;
			// Calculate additional costs
			if (beforeOrAfter == 1){
				distanceCosts += params->timeCost.get(currentNode->cour, nextNode->cour) + params->timeCost.get(nextNode->cour, nextNode->next->servedBy) - params->timeCost.get(currentNode->cour, nextNode->servedBy);
				routeUTwData1 = MergeTWDataRecursiveNewIndicesNewTWNoParcelPoint(routeUTwData1, nextNode->twData, currentNode->cour, nextNode->servedBy, routeUTwData1.earliestArrival,routeUTwData1.latestArrival,params->cli[nextNode->cour].earliestArrival,params->cli[nextNode->cour].latestArrival);
				routeUTwData = MergeTWDataRecursiveNewIndicesNewTW(routeUTwData1, nextNode->next->postfixTwData, nextNode->cour, nextNode->next->servedBy, routeUTwData1.earliestArrival, routeUTwData1.latestArrival, nextNode->next->postfixTwData.earliestArrival, nextNode->next->postfixTwData.latestArrival);
			} else{
				distanceCosts += params->timeCost.get(currentNode->cour, nextNode->cour) + params->timeCost.get(nextNode->cour, nodeX->servedBy) - params->timeCost.get(currentNode->cour, nodeX->servedBy);
				routeUTwData2 = MergeTWDataRecursiveNewIndicesNewTW(routeUTwData2, nextNode->twData, nodeU->cour, nextNode->cour, routeUTwData2.earliestArrival, routeUTwData2.latestArrival,params->cli[nextNode->cour].earliestArrival,params->cli[nextNode->cour].latestArrival);
				routeUTwData = MergeTWDataRecursiveNewIndicesNewTWNoParcelPoint(routeUTwData2, nextNode->next->postfixTwData, nextNode->cour, nextNode->next->servedBy, routeUTwData2.earliestArrival, routeUTwData2.latestArrival,nextNode->next->postfixTwData.earliestArrival,nextNode->next->postfixTwData.latestArrival);
			}
			discountCosts -= params->priceSensitivity;
			penaltyCosts = penaltyExcessLoadParcelPoint(parcelPoint->totalClientsServed - counter, params->cellsPerParcelPoint[parcelPoint->cour-params->nbClients-1]) - penaltyExcessLoadParcelPoint(parcelPoint->totalClientsServed, params->cellsPerParcelPoint[parcelPoint->cour-params->nbClients-1]);
			timeWindowCosts = penaltyTimeWindows(routeUTwData);

			additionalCosts = distanceCosts + discountCosts + penaltyCosts + timeWindowCosts - routeU->penalty + penaltyExcessLoadVehicle(routeU->load);
			if (additionalCosts < -MY_EPSILON){
				moveList.push_back(nextNode);
				costBenefits = true;
			} else{
				moveList.push_back(nextNode);
				counter += 1;
				currentNode = nextNode;
			}
		}
		// Return false when there are no cost benefits
		if (!costBenefits) return false;
		moveMultipleClientsBackToClients(moveList, parcelPoint, I, beforeOrAfter);
		nbMoves++; // Increment move counter before updating route data
		searchCompleted = false;
		updateRouteData(routeU);
		return true;
		}
}


bool LocalSearch::MoveSingleClient()
{
	// If U already comes directly after V, this move has no effect
	if (nodeUIndex == nodeYIndex) return false;
	double costSuppU = params->timeCost.get(nodeUPrevIndex, nodeXIndex) - params->timeCost.get(nodeUPrevIndex, nodeUIndex) - params->timeCost.get(nodeUIndex, nodeXIndex);
	double costSuppV = params->timeCost.get(nodeVIndex, nodeUIndex) + params->timeCost.get(nodeUIndex, nodeYIndex) - params->timeCost.get(nodeVIndex, nodeYIndex);
	TimeWindowData routeUTwData, routeVTwData;

	if (routeU != routeV)
	{
		if (!routeULoadPenalty && !routeUTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
		{
			return false;
		}
		if (nodeV->cour == 0 && nodeV->next->cour ==0){
			costSuppV += params->costPerVehicle;
		}
		routeUTwData = MergeTWDataRecursive(nodeUPrev->prefixTwData, nodeXReal->postfixTwData);
		routeVTwData = MergeTWDataRecursive(nodeV->prefixTwData, getRouteSegmentTwData(nodeUPrevTW, nodeU), nodeYReal->postfixTwData);

		costSuppU += penaltyExcessLoadVehicle(routeU->load - loadU)
			+ penaltyTimeWindows(routeUTwData)
			- routeU->penalty;

		costSuppV += penaltyExcessLoadVehicle(routeV->load + loadU)
			+ penaltyTimeWindows(routeVTwData)
			- routeV->penalty;

	}
	else
	{
		if (!routeUTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
		{
			return false;
		}

		// Move within the same route
		if (nodeU->position < nodeV->position)
		{
			// Edge case V directly after U, so X == V, this works
			// start - ... - UPrev - X - ... - V - U - Y - ... - end
			routeUTwData = MergeTWDataRecursive(nodeUPrev->prefixTwData, getRouteSegmentTwData(nodeU->next, nodeV), getRouteSegmentTwData(nodeUPrevTW, nodeU), nodeV->next->postfixTwData);
		}
		else
		{
			getRouteSegmentTwData(nodeY, nodeUPrev);
			// Edge case U directly after V is excluded from beginning of function
			// start - ... - V - U - Y - ... - UPrev - X - ... - end
			routeUTwData = MergeTWDataRecursive(nodeV->prefixTwData, getRouteSegmentTwData(nodeUPrevTW, nodeU), getRouteSegmentTwData(nodeV->next, nodeUPrev), nodeU->next->postfixTwData);
		}


		// Compute new total penalty
		costSuppU += penaltyExcessLoadVehicle(routeU->load)
			+ penaltyTimeWindows(routeUTwData)
			- routeU->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	insertNode(nodeU, nodeV);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (routeU != routeV) updateRouteData(routeV);

	return true;
}

bool LocalSearch::MoveTwoClients()
{
	if (nodeU == nodeY || nodeV == nodeX || nodeU->servedBy == nodeY->servedBy || nodeV->servedBy == nodeX->servedBy|| nodeUPrev->servedBy == nodeX->servedBy|| nodeX->isDepot) return false;

	double costSuppU = params->timeCost.get(nodeUPrevIndex, nodeXNextIndex) - params->timeCost.get(nodeUPrevIndex, nodeUIndex) - params->timeCost.get(nodeXIndex, nodeXNextIndex);
	double costSuppV = params->timeCost.get(nodeVIndex, nodeUIndex) + params->timeCost.get(nodeXIndex, nodeYIndex) - params->timeCost.get(nodeVIndex, nodeYIndex);
	TimeWindowData routeUTwData, routeVTwData;

	if (routeU != routeV)
	{
		if (!routeULoadPenalty && !routeUTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
		{
			return false;
		}
		if (nodeV->cour == 0 && nodeV->next->cour ==0){
			costSuppV += params->costPerVehicle;
		}

		routeUTwData = MergeTWDataRecursive(nodeUPrev->prefixTwData, nodeXReal->postfixTwData);
		routeVTwData = MergeTWDataRecursive(nodeV->prefixTwData, getRouteSegmentTwData(nodeUPrevTW, nodeX), nodeYReal->postfixTwData);

		costSuppU += penaltyExcessLoadVehicle(routeU->load - loadU - loadX)
			+ penaltyTimeWindows(routeUTwData)
			- routeU->penalty;

		costSuppV += penaltyExcessLoadVehicle(routeV->load + loadU + loadX)
			+ penaltyTimeWindows(routeVTwData)
			- routeV->penalty;
	}
	else
	{
		if (!routeUTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
		{
			return false;
		}

		// Move within the same route
		if (nodeU->position < nodeV->position)
		{
			// Edge case V directly after U, so X == V is excluded, V directly after X so XNext == V works
			// start - ... - UPrev - XNext - ... - V - U - X - Y - ... - end
			routeUTwData = MergeTWDataRecursive(nodeUPrev->prefixTwData, getRouteSegmentTwData(nodeXNext, nodeV), getRouteSegmentTwData(nodeUPrevTW, nodeX), nodeV->next->postfixTwData);
		}
		else
		{
			// Edge case U directly after V is excluded from beginning of function
			// start - ... - V - U - X - Y - ... - UPrev - XNext - ... - end
			routeUTwData = MergeTWDataRecursive(nodeV->prefixTwData, getRouteSegmentTwData(nodeUPrevTW, nodeX), getRouteSegmentTwData(nodeV->next, nodeUPrev), nodeXNext->postfixTwData);
		}

		// Compute new total penalty
		costSuppU += penaltyExcessLoadVehicle(routeU->load)
			+ penaltyTimeWindows(routeUTwData)
			- routeU->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	insertNode(nodeU, nodeV);
	insertNode(nodeX, nodeU);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (routeU != routeV) updateRouteData(routeV);

	return true;
}

bool LocalSearch::MoveTwoClientsReversed()
{
	if (nodeU == nodeY || nodeX == nodeV || nodeU->servedBy == nodeY->servedBy || nodeV->servedBy == nodeX->servedBy || nodeX->isDepot) return false;

	double costSuppU = params->timeCost.get(nodeUPrevIndex, nodeXNextIndex) - params->timeCost.get(nodeUPrevIndex, nodeUIndex) - params->timeCost.get(nodeUIndex, nodeXIndex) - params->timeCost.get(nodeXIndex, nodeXNextIndex);
	double costSuppV = params->timeCost.get(nodeVIndex, nodeXIndex) + params->timeCost.get(nodeXIndex, nodeUIndex) + params->timeCost.get(nodeUIndex, nodeYIndex) - params->timeCost.get(nodeVIndex, nodeYIndex);
	TimeWindowData routeUTwData, routeVTwData;

	if (routeU != routeV)
	{
		if (!routeULoadPenalty && !routeUTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
		{
			return false;
		}

		if (nodeV->cour == 0 && nodeV->next->cour ==0){
			costSuppV += params->costPerVehicle;
		}

		routeUTwData = MergeTWDataRecursive(nodeUPrev->prefixTwData, nodeX->next->postfixTwData);
		routeVTwData = MergeTWDataRecursive(nodeV->prefixTwData, getRouteSegmentTwData(nodeU->next, nodeU), nodeV->next->postfixTwData);

		costSuppU += penaltyExcessLoadVehicle(routeU->load - loadU - loadX)
			+ penaltyTimeWindows(routeUTwData)
			- routeU->penalty;

		costSuppV += penaltyExcessLoadVehicle(routeV->load + loadU + loadX)
			+ penaltyTimeWindows(routeVTwData)
			- routeV->penalty;
	}
	else
	{
		if (!routeUTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
		{
			return false;
		}

		// Move within the same route
		if (nodeU->position < nodeV->position)
		{
			// Edge case V directly after U, so X == V is excluded, V directly after X so XNext == V works
			// start - ... - UPrev - XNext - ... - V - X - U - Y - ... - end
			routeUTwData = MergeTWDataRecursive(nodeUPrev->prefixTwData, getRouteSegmentTwData(nodeX->next, nodeV), getRouteSegmentTwData(nodeU->next, nodeU), nodeV->next->postfixTwData);
		}
		else
		{
			// Edge case U directly after V is excluded from beginning of function
			// start - ... - V - X - U - Y - ... - UPrev - XNext - ... - end
			routeUTwData = MergeTWDataRecursive(nodeV->prefixTwData, getRouteSegmentTwData(nodeU->next, nodeU), getRouteSegmentTwData(nodeV->next, nodeUPrev), nodeX->next->postfixTwData);
		}

		// Compute new total penalty
		costSuppU += penaltyExcessLoadVehicle(routeU->load)
			+ penaltyTimeWindows(routeUTwData)
			- routeU->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	insertNode(nodeX, nodeV);
	insertNode(nodeU, nodeX);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (routeU != routeV) updateRouteData(routeV);

	return true;
}

bool LocalSearch::SwapTwoSingleClients()
{
	if (nodeUIndex == nodeVPrevIndex || nodeUIndex == nodeYIndex) return false;

	double costSuppU = params->timeCost.get(nodeUPrevIndex, nodeVIndex) + params->timeCost.get(nodeVIndex, nodeXIndex) - params->timeCost.get(nodeUPrevIndex, nodeUIndex) - params->timeCost.get(nodeUIndex, nodeXIndex);
	double costSuppV = params->timeCost.get(nodeVPrevIndex, nodeUIndex) + params->timeCost.get(nodeUIndex, nodeYIndex) - params->timeCost.get(nodeVPrevIndex, nodeVIndex) - params->timeCost.get(nodeVIndex, nodeYIndex);
	TimeWindowData routeUTwData, routeVTwData;

	if (routeU != routeV)
	{
		if (!routeULoadPenalty && !routeUTimeWarp && !routeVLoadPenalty && !routeVTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
		{
			return false;
		}

		routeUTwData = MergeTWDataRecursive(nodeUPrev->prefixTwData, getRouteSegmentTwData(nodeVPrevTW, nodeV), nodeXReal->postfixTwData);
		routeVTwData = MergeTWDataRecursive(nodeVPrev->prefixTwData, getRouteSegmentTwData(nodeUPrevTW, nodeU), nodeYReal->postfixTwData);

		costSuppU += penaltyExcessLoadVehicle(routeU->load + loadV - loadU)
			+ penaltyTimeWindows(routeUTwData)
			- routeU->penalty;

		costSuppV += penaltyExcessLoadVehicle(routeV->load + loadU - loadV)
			+ penaltyTimeWindows(routeVTwData)
			- routeV->penalty;
	}
	else
	{
		if (!routeUTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
		{
			return false;
		}

		// Swap within the same route
		if (nodeU->position < nodeV->position)
		{
			// Edge case V directly after U, so X == V is excluded, V directly after X so XNext == V works
			// start - ... - UPrev - V - X - ... - VPrev - U - Y - ... - end
			routeUTwData = MergeTWDataRecursive(nodeUPrev->prefixTwData, getRouteSegmentTwData(nodeVPrevTW, nodeV), getRouteSegmentTwData(nodeXReal, nodeVPrev), getRouteSegmentTwData(nodeUPrevTW, nodeU), nodeYReal->postfixTwData);
		}
		else
		{
			// Edge case U directly after V is excluded from beginning of function
			// start - ... - VPrev - U - Y - ... - UPrev - V - X - ... - end
			routeUTwData = MergeTWDataRecursive(nodeVPrev->prefixTwData, getRouteSegmentTwData(nodeUPrevTW, nodeU), getRouteSegmentTwData(nodeYReal, nodeUPrev), getRouteSegmentTwData(nodeVPrevTW, nodeV), nodeXReal->postfixTwData);
		}

		// Compute new total penalty
		costSuppU += penaltyExcessLoadVehicle(routeU->load)
			+ penaltyTimeWindows(routeUTwData)
			- routeU->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	swapNode(nodeU, nodeV);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (routeU != routeV) updateRouteData(routeV);

	return true;
}

bool LocalSearch::SwapTwoClientsForOne()
{
	if (nodeUIndex == nodeVPrevIndex || nodeXIndex == nodeVPrevIndex || nodeUIndex == nodeYIndex || nodeXIndex == nodeVIndex|| nodeX->isDepot) return false;

	double costSuppU = params->timeCost.get(nodeUPrevIndex, nodeVIndex) + params->timeCost.get(nodeVIndex, nodeXNextIndex) - params->timeCost.get(nodeUPrevIndex, nodeUIndex) - params->timeCost.get(nodeXIndex, nodeXNextIndex);
	double costSuppV = params->timeCost.get(nodeVPrevIndex, nodeUIndex) + params->timeCost.get(nodeXIndex, nodeYIndex) - params->timeCost.get(nodeVPrevIndex, nodeVIndex) - params->timeCost.get(nodeVIndex, nodeYIndex);
	TimeWindowData routeUTwData, routeVTwData;

	if (routeU != routeV)
	{
		if (!routeULoadPenalty && !routeUTimeWarp && !routeVLoadPenalty && !routeVTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
		{
			return false;
		}

		routeUTwData = MergeTWDataRecursive(nodeUPrev->prefixTwData, getRouteSegmentTwData(nodeVPrevTW, nodeV), nodeXReal->postfixTwData);
		routeVTwData = MergeTWDataRecursive(nodeVPrev->prefixTwData, getRouteSegmentTwData(nodeUPrevTW, nodeX), nodeYReal->postfixTwData);

		costSuppU += penaltyExcessLoadVehicle(routeU->load + loadV - loadU - loadX)
			+ penaltyTimeWindows(routeUTwData)
			- routeU->penalty;

		costSuppV += penaltyExcessLoadVehicle(routeV->load + loadU + loadX - loadV)
			+ penaltyTimeWindows(routeVTwData)
			- routeV->penalty;
	}
	else
	{
		if (!routeUTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
		{
			return false;
		}

		// Swap within the same route
		if (nodeU->position < nodeV->position)
		{
			// start - ... - UPrev - V - XNext - ... - VPrev - U - X - Y - ... - end
			routeUTwData = MergeTWDataRecursive(nodeUPrev->prefixTwData, getRouteSegmentTwData(nodeVPrevTW, nodeV), getRouteSegmentTwData(nodeX->next, nodeVPrev), getRouteSegmentTwData(nodeUPrevTW, nodeX), nodeYReal->postfixTwData);
		}
		else
		{
			// start - ... - VPrev - U - X - Y - ... - UPrev - V - XNext - ... - end
			routeUTwData = MergeTWDataRecursive(nodeVPrev->prefixTwData, getRouteSegmentTwData(nodeUPrevTW, nodeX), getRouteSegmentTwData(nodeYReal, nodeUPrev), getRouteSegmentTwData(nodeVPrevTW, nodeV), nodeXReal->postfixTwData);
		}

		// Compute new total penalty
		costSuppU += penaltyExcessLoadVehicle(routeU->load)
			+ penaltyTimeWindows(routeUTwData)
			- routeU->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	// Note: next two lines are a bit inefficient but we only update occasionally and updateRouteData is much more costly anyway, efficient checks are more important
	swapNode(nodeU, nodeV);
	insertNode(nodeX, nodeU);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (routeU != routeV) updateRouteData(routeV);
	return true;
}

bool LocalSearch::SwapTwoClientPairs()
{
	if (nodeX->isDepot || nodeY->isDepot || nodeYIndex == nodeUPrevIndex|| nodeUIndex == nodeYIndex || nodeXIndex == nodeVIndex || nodeVIndex == nodeX->next->servedBy) return false;

	double costSuppU = params->timeCost.get(nodeUPrevIndex, nodeVIndex) + params->timeCost.get(nodeYIndex, nodeXNextIndex) - params->timeCost.get(nodeUPrevIndex, nodeUIndex) - params->timeCost.get(nodeXIndex, nodeXNextIndex);
	double costSuppV = params->timeCost.get(nodeVPrevIndex, nodeUIndex) + params->timeCost.get(nodeXIndex, nodeYNextIndex) - params->timeCost.get(nodeVPrevIndex, nodeVIndex) - params->timeCost.get(nodeYIndex, nodeYNextIndex);
	TimeWindowData routeUTwData, routeVTwData;

	if (routeU != routeV)
	{
		if (!routeULoadPenalty && !routeUTimeWarp && !routeVLoadPenalty && !routeVTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
		{
			return false;
		}

		routeUTwData = MergeTWDataRecursive(nodeUPrev->prefixTwData, getRouteSegmentTwData(nodeVPrevTW, nodeY), nodeX->next->postfixTwData);
		routeVTwData = MergeTWDataRecursive(nodeVPrev->prefixTwData, getRouteSegmentTwData(nodeUPrevTW, nodeX), nodeY->next->postfixTwData);

		costSuppU += penaltyExcessLoadVehicle(routeU->load + loadV + loadY - loadU - loadX)
			+ penaltyTimeWindows(routeUTwData)
			- routeU->penalty;

		costSuppV += penaltyExcessLoadVehicle(routeV->load + loadU + loadX - loadV - loadY)
			+ penaltyTimeWindows(routeVTwData)
			- routeV->penalty;
	}
	else
	{
		if (!routeUTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
		{
			return false;
		}

		// Swap within the same route
		if (nodeU->position < nodeV->position)
		{
			// start - ... - UPrev - V - Y - XNext - ... - VPrev - U - X - YNext  - ... - end
			routeUTwData = MergeTWDataRecursive(nodeUPrev->prefixTwData, getRouteSegmentTwData(nodeVPrevTW, nodeY), getRouteSegmentTwData(nodeX->next, nodeVPrev), getRouteSegmentTwData(nodeUPrevTW, nodeX), nodeY->next->postfixTwData);
		}
		else
		{
			// start - ... - VPrev - U - X - YNext - ... - UPrev - V - Y - XNext - ... - end
			routeUTwData = MergeTWDataRecursive(nodeVPrev->prefixTwData, getRouteSegmentTwData(nodeUPrevTW, nodeX), getRouteSegmentTwData(nodeY->next, nodeUPrev), getRouteSegmentTwData(nodeVPrevTW, nodeY), nodeX->next->postfixTwData);
		}

		// Compute new total penalty
		costSuppU += penaltyExcessLoadVehicle(routeU->load)
			+ penaltyTimeWindows(routeUTwData)
			- routeU->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	swapNode(nodeU, nodeV);
	swapNode(nodeX, nodeY);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (routeU != routeV) updateRouteData(routeV);


	return true;
}

bool LocalSearch::TwoOptWithinTrip()
{
	if (nodeU->position >= nodeV->position - 1 || nodeX->servedBy == nodeV->servedBy) return false;

	double cost = params->timeCost.get(nodeUIndex, nodeVIndex) + params->timeCost.get(nodeXIndex, nodeYIndex) - params->timeCost.get(nodeUIndex, nodeXIndex) - params->timeCost.get(nodeVIndex, nodeYIndex) + nodeV->cumulatedReversalDistance - nodeU->next->cumulatedReversalDistance;
	double cost2 = params->timeCost.get(nodeUIndex, nodeVIndex) + params->timeCost.get(nodeXIndex, nodeYIndex) - params->timeCost.get(nodeUIndex, nodeXIndex) - params->timeCost.get(nodeVIndex, nodeYIndex) + nodeV->cumulatedReversalDistance - nodeU->next->cumulatedReversalDistance;
	if (!routeUTimeWarp && cost > -MY_EPSILON)
	{
		return false;
	}

	TimeWindowData routeTwData = nodeX->prefixTwData;
	Node* itRoute = nodeV;
	while (itRoute != nodeU)
	{
		routeTwData = MergeTWDataRecursive(routeTwData, itRoute->twData);
		itRoute = itRoute->prev;
	}
	routeTwData = MergeTWDataRecursive(routeTwData, nodeYReal->postfixTwData);

	// Compute new total penalty
	cost += penaltyExcessLoadVehicle(routeU->load)
		+ penaltyTimeWindows(routeTwData)
		- routeU->penalty;

	
	if (cost > -MY_EPSILON) return false;
	double bef = getCostSol().penalizedCost;
	double olddi = getCostSol().distance;
	itRoute = nodeV;
	Node* insertionPoint = nodeU;
	while (itRoute != nodeX) // No need to move x, we pivot around it
	{
		Node* current = itRoute;
		itRoute = itRoute->prev;
		insertSingleNode(current, insertionPoint);
		insertionPoint = current;
	}

	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (bef < getCostSol().penalizedCost)
	{
		std::cout<<"Mistake"<<std::endl;
		printRoute(routeU);
		printRoute(routeU, false);
			std::cout<<nodeV->servedBy << " " << nodeYReal->servedBy<<std::endl;
			std::cout<<nodeV->cour << " " << nodeYReal->cour<<std::endl;
		std::cout<<nodeU->servedBy<< " " << nodeXReal->servedBy<<std::endl;
		std::cout<<cost2<<std::endl;
		std::cout<<olddi<<std::endl;
		std::cout<<getCostSol().distance<<std::endl;
	}
	return true;
}

bool LocalSearch::TwoOptBetweenTrips()
{
	double costSuppU = params->timeCost.get(nodeUIndex, nodeYIndex) - params->timeCost.get(nodeUIndex, nodeXIndex);
	double costSuppV = params->timeCost.get(nodeVIndex, nodeXIndex) - params->timeCost.get(nodeVIndex, nodeYIndex);

	if (!routeULoadPenalty && !routeUTimeWarp && !routeVLoadPenalty && !routeVTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
	{
		return false;
	}

	TimeWindowData routeUTwData, routeVTwData;
	if (nodeV->cour == 0 && nodeV->next->cour ==0){
		costSuppV += params->costPerVehicle;
	}
	routeUTwData = MergeTWDataRecursive(nodeUPrev->prefixTwData,getRouteSegmentTwData(nodeUPrevTW, nodeU), nodeYReal->postfixTwData);
	routeVTwData = MergeTWDataRecursive(nodeVPrev->prefixTwData, getRouteSegmentTwData(nodeVPrevTW, nodeV), nodeXReal->postfixTwData);

	costSuppU += penaltyExcessLoadVehicle(nodeU->cumulatedLoad + routeV->load - nodeV->cumulatedLoad)
		+ penaltyTimeWindows(routeUTwData)
		- routeU->penalty;

	costSuppV += penaltyExcessLoadVehicle(nodeV->cumulatedLoad + routeU->load - nodeU->cumulatedLoad)
		+ penaltyTimeWindows(routeVTwData)
		- routeV->penalty;

	if (costSuppU + costSuppV > -MY_EPSILON) return false;

	Node* itRouteV = nodeYReal;
	Node* insertLocation = nodeU;
	while (!itRouteV->isDepot)
	{
		Node* current = itRouteV;
		itRouteV = itRouteV->next;
		insertSingleNode(current, insertLocation);
		insertLocation = current;
	}

	Node* itRouteU = nodeXReal;
	insertLocation = nodeV;
	while (!itRouteU->isDepot)
	{
		Node* current = itRouteU;
		itRouteU = itRouteU->next;
		insertSingleNode(current, insertLocation);
		insertLocation = current;
	}

	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	updateRouteData(routeV);

	return true;
}

bool LocalSearch::swapStar(const bool withTW)
{
	SwapStarElement myBestSwapStar;

	if (!bestInsertInitializedForRoute[routeU->cour])
	{
		bestInsertInitializedForRoute[routeU->cour] = true;
		for (int i = 1; i <= params->nbClients + params->nbParcelPoints; i++)
		{
			bestInsertClient[routeU->cour][i].whenLastCalculated = -1;
			bestInsertClientTW[routeU->cour][i].whenLastCalculated = -1;
		}
	}
	if (!bestInsertInitializedForRoute[routeV->cour])
	{
		bestInsertInitializedForRoute[routeV->cour] = true;
		for (int i = 1; i <= params->nbClients + params->nbParcelPoints; i++)
		{
			bestInsertClient[routeV->cour][i].whenLastCalculated = -1;
			bestInsertClientTW[routeV->cour][i].whenLastCalculated = -1;
		}
	}

	// Preprocessing insertion costs
	if (withTW)
	{
		preprocessInsertionsWithTW(routeU, routeV);
		preprocessInsertionsWithTW(routeV, routeU);
	}
	else
	{
		preprocessInsertions(routeU, routeV);
		preprocessInsertions(routeV, routeU);
	}

	// Evaluating the moves
	for (nodeU = routeU->depot->next; !nodeU->isDepot; nodeU = nodeU->next)
	{
		if (nodeU->servedBy != nodeU->cour) continue;
		for (nodeV = routeV->depot->next; !nodeV->isDepot; nodeV = nodeV->next)
		{
			if (nodeV->servedBy != nodeV->cour) continue;
			// We cannot determine impact on timewarp without adding too much complexity (O(n^3) instead of O(n^2))
			const double loadPenU = penaltyExcessLoadVehicle(routeU->load + params->cli[nodeV->cour].demand - params->cli[nodeU->cour].demand);
			const double loadPenV = penaltyExcessLoadVehicle(routeV->load + params->cli[nodeU->cour].demand - params->cli[nodeV->cour].demand);
			const double deltaLoadPen = loadPenU + loadPenV - penaltyExcessLoadVehicle(routeU->load) - penaltyExcessLoadVehicle(routeV->load);
			const int deltaRemoval = withTW ? nodeU->deltaRemovalTW + nodeV->deltaRemovalTW : nodeU->deltaRemoval + nodeV->deltaRemoval;

			// Quick filter: possibly early elimination of many SWAP* due to the capacity constraints/penalties and bounds on insertion costs
			if (deltaLoadPen + deltaRemoval <= 0)
			{
				SwapStarElement mySwapStar;
				mySwapStar.U = nodeU;
				mySwapStar.V = nodeV;

				int extraV, extraU;
				if (withTW){
					// Evaluate best reinsertion cost of U in the route of V where V has been removed
					extraV = getCheapestInsertSimultRemovalWithTW(nodeU, nodeV, mySwapStar.bestPositionU);

					// Evaluate best reinsertion cost of V in the route of U where U has been removed
					extraU = getCheapestInsertSimultRemovalWithTW(nodeV, nodeU, mySwapStar.bestPositionV);

				} else {
					// Evaluate best reinsertion cost of U in the route of V where V has been removed
					extraV = getCheapestInsertSimultRemoval(nodeU, nodeV, mySwapStar.bestPositionU);

					// Evaluate best reinsertion cost of V in the route of U where U has been removed
					extraU = getCheapestInsertSimultRemoval(nodeV, nodeU, mySwapStar.bestPositionV);
				}
				// Evaluating final cost
				mySwapStar.moveCost = deltaLoadPen + deltaRemoval + extraU + extraV;

				if (mySwapStar.moveCost < myBestSwapStar.moveCost)
				{
					myBestSwapStar = mySwapStar;
					myBestSwapStar.loadPenU = loadPenU;
					myBestSwapStar.loadPenV = loadPenV;
				}
			}
		}
	}

	if (!myBestSwapStar.bestPositionU || !myBestSwapStar.bestPositionV)
	{
		return false;
	}

	// Compute actual cost including TimeWarp penalty
	double costSuppU = params->timeCost.get(myBestSwapStar.bestPositionV->servedBy, myBestSwapStar.V->servedBy) -
		params->timeCost.get(myBestSwapStar.U->prev->servedBy, myBestSwapStar.U->servedBy) -
		params->timeCost.get(myBestSwapStar.U->servedBy, myBestSwapStar.U->next->servedBy);
	double costSuppV = params->timeCost.get(myBestSwapStar.bestPositionU->servedBy, myBestSwapStar.U->servedBy) -
		params->timeCost.get(myBestSwapStar.V->prev->servedBy, myBestSwapStar.V->servedBy) -
		params->timeCost.get(myBestSwapStar.V->servedBy, myBestSwapStar.V->next->servedBy);

	if (myBestSwapStar.bestPositionV == myBestSwapStar.U->prev)
	{
		// Insert in place of U
		costSuppU += params->timeCost.get(myBestSwapStar.V->servedBy, myBestSwapStar.U->next->servedBy);
	}
	else
	{
		costSuppU += params->timeCost.get(myBestSwapStar.V->servedBy, myBestSwapStar.bestPositionV->next->servedBy) +
			params->timeCost.get(myBestSwapStar.U->prev->servedBy, myBestSwapStar.U->next->servedBy) -
			params->timeCost.get(myBestSwapStar.bestPositionV->servedBy, myBestSwapStar.bestPositionV->next->servedBy);
	}

	if (myBestSwapStar.bestPositionU == myBestSwapStar.V->prev)
	{
		// Insert in place of V
		costSuppV += params->timeCost.get(myBestSwapStar.U->servedBy, myBestSwapStar.V->next->servedBy);
	}
	else
	{
		costSuppV += params->timeCost.get(myBestSwapStar.U->servedBy, myBestSwapStar.bestPositionU->next->servedBy) +
			params->timeCost.get(myBestSwapStar.V->prev->servedBy, myBestSwapStar.V->next->servedBy) -
			params->timeCost.get(myBestSwapStar.bestPositionU->servedBy, myBestSwapStar.bestPositionU->next->servedBy);;
	}

	TimeWindowData routeUTwData, routeVTwData;
	// It is not possible to have bestPositionU == V or bestPositionV == U, so the positions are always strictly different
	if (myBestSwapStar.bestPositionV->position == myBestSwapStar.U->position - 1)
	{
		// Special case
		routeUTwData = MergeTWDataRecursive(myBestSwapStar.bestPositionV->prefixTwData, myBestSwapStar.V->twData, myBestSwapStar.U->next->postfixTwData);
	}
	else if (myBestSwapStar.bestPositionV->position < myBestSwapStar.U->position)
	{
		routeUTwData = MergeTWDataRecursive(myBestSwapStar.bestPositionV->prefixTwData,
			myBestSwapStar.V->twData,
			getRouteSegmentTwData(myBestSwapStar.bestPositionV->next, myBestSwapStar.U->prev),
			myBestSwapStar.U->next->postfixTwData);
	}
	else
	{
		routeUTwData = MergeTWDataRecursive(myBestSwapStar.U->prev->prefixTwData,
			getRouteSegmentTwData(myBestSwapStar.U->next, myBestSwapStar.bestPositionV),
			myBestSwapStar.V->twData,
			myBestSwapStar.bestPositionV->next->postfixTwData);
	}

	if (myBestSwapStar.bestPositionU->position == myBestSwapStar.V->position - 1)
	{
		// Special case
		routeVTwData = MergeTWDataRecursive(myBestSwapStar.bestPositionU->prefixTwData, myBestSwapStar.U->twData, myBestSwapStar.V->next->postfixTwData);
	}
	else if (myBestSwapStar.bestPositionU->position < myBestSwapStar.V->position)
	{
		routeVTwData = MergeTWDataRecursive(myBestSwapStar.bestPositionU->prefixTwData,
			myBestSwapStar.U->twData,
			getRouteSegmentTwData(myBestSwapStar.bestPositionU->next, myBestSwapStar.V->prev),
			myBestSwapStar.V->next->postfixTwData);
	}
	else
	{
		routeVTwData = MergeTWDataRecursive(myBestSwapStar.V->prev->prefixTwData,
			getRouteSegmentTwData(myBestSwapStar.V->next, myBestSwapStar.bestPositionU),
			myBestSwapStar.U->twData,
			myBestSwapStar.bestPositionU->next->postfixTwData);
	}

	costSuppU += myBestSwapStar.loadPenU + penaltyTimeWindows(routeUTwData) - routeU->penalty;

	costSuppV += myBestSwapStar.loadPenV + penaltyTimeWindows(routeVTwData) - routeV->penalty;

	if (costSuppU + costSuppV > -MY_EPSILON)
	{
		return false;
	}

	// Applying the best move in case of improvement
	insertSingleNode(myBestSwapStar.U, myBestSwapStar.bestPositionU);
	insertSingleNode(myBestSwapStar.V, myBestSwapStar.bestPositionV);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	updateRouteData(routeV);
	return true;
}

bool LocalSearch::RelocateStar()
{
	double bestCost = 0;
	Node* insertionPoint = nullptr;
	Node* nodeToInsert = nullptr;
	for (nodeU = routeU->depot->next; !nodeU->isDepot; nodeU = nodeU->next)
	{
		if (nodeU->servedBy == nodeU->next->servedBy) continue;
		setLocalVariablesRouteU();

		const TimeWindowData routeUTwData = MergeTWDataRecursive(nodeUPrev->prefixTwData, nodeXReal->postfixTwData);
		const double costSuppU = params->timeCost.get(nodeUPrevIndex, nodeXIndex)
			- params->timeCost.get(nodeUPrevIndex, nodeUIndex)
			- params->timeCost.get(nodeUIndex, nodeXIndex)
			+ penaltyExcessLoadVehicle(routeU->load - loadU)
			+ penaltyTimeWindows(routeUTwData)
			- routeU->penalty;

		for (Node* V = routeV->depot->next; !V->isDepot; V = V->next)
		{
			if (V->servedBy == V->next->servedBy) continue;
			const TimeWindowData routeVTwData = MergeTWDataRecursive(V->prefixTwData, getRouteSegmentTwData(nodeUPrevTW, nodeU), V->next->postfixTwData);
			double costSuppV = params->timeCost.get(V->servedBy, nodeUIndex)
				+ params->timeCost.get(nodeUIndex, V->next->servedBy)
				- params->timeCost.get(V->servedBy, V->next->servedBy)
				+ penaltyExcessLoadVehicle(routeV->load + loadU)
				+ penaltyTimeWindows(routeVTwData)
				- routeV->penalty;
			if (costSuppU + costSuppV < bestCost - MY_EPSILON)
			{
				bestCost = costSuppU + costSuppV;
				insertionPoint = V;
				nodeToInsert = nodeU;
			}
		}
	}

	if (!insertionPoint) return false;

	double bef = getCostSol().penalizedCost;
	double olddi = penaltyTimeWindows(routeU->twData);
	double olddiV = penaltyTimeWindows(routeV->twData);

	routeU = nodeToInsert->route;
	insertNode(nodeToInsert, insertionPoint);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	updateRouteData(insertionPoint->route);

	if (bef < getCostSol().penalizedCost && routeU != routeV)
	{
		std::cout<<"Mistake8"<<std::endl;
		std::cout<<olddi<<std::endl;
		std::cout<<penaltyTimeWindows(routeU->twData)<<std::endl;
		std::cout<<olddiV<<std::endl;
		std::cout<<penaltyTimeWindows(routeV->twData)<<std::endl;
		std::cout<<bef << " " << getCostSol().penalizedCost <<std::endl;
	}
	return true;
}

int LocalSearch::getCheapestInsertSimultRemoval(Node* U, Node* V, Node*& bestPosition)
{
	ThreeBestInsert* myBestInsert = &bestInsertClient[V->route->cour][U->servedBy];
	bool found = false;
	// Find best insertion in the route such that V is not next or pred (can only belong to the top three locations)
	bestPosition = myBestInsert->bestLocation[0];
	int bestCost = myBestInsert->bestCost[0];
	found = (bestPosition != V && bestPosition->next != V);
	if (!found && myBestInsert->bestLocation[1] != nullptr)
	{
		bestPosition = myBestInsert->bestLocation[1];
		bestCost = myBestInsert->bestCost[1];
		found = (bestPosition != V && bestPosition->next != V);
		if (!found && myBestInsert->bestLocation[2] != nullptr)
		{
			bestPosition = myBestInsert->bestLocation[2];
			bestCost = myBestInsert->bestCost[2];
			found = true;
		}
	}
	// Compute insertion in the place of V
	int deltaCost = params->timeCost.get(V->prev->servedBy, U->servedBy) + params->timeCost.get(U->servedBy, V->next->servedBy) - params->timeCost.get(V->prev->servedBy, V->next->servedBy);
	if (!found || deltaCost < bestCost)
	{
		bestPosition = V->prev;
		bestCost = deltaCost;
	}
	return bestCost;
}

// TODO make this double as cost??
int LocalSearch::getCheapestInsertSimultRemovalWithTW(Node* U, Node* V, Node*& bestPosition)
{
	// TODO ThreeBestInsert must also use double as cost?
	ThreeBestInsert* myBestInsert = &bestInsertClientTW[V->route->cour][U->servedBy];
	bool found = false;

	// Find best insertion in the route such that V is not next or pred (can only belong to the top three locations)
	bestPosition = myBestInsert->bestLocation[0];
	int bestCost = myBestInsert->bestCost[0];
	found = (bestPosition != V && bestPosition->next != V);
	if (!found && myBestInsert->bestLocation[1] != nullptr)
	{
		bestPosition = myBestInsert->bestLocation[1];
		bestCost = myBestInsert->bestCost[1];
		found = (bestPosition != V && bestPosition->next != V);
		if (!found && myBestInsert->bestLocation[2] != nullptr)
		{
			bestPosition = myBestInsert->bestLocation[2];
			bestCost = myBestInsert->bestCost[2];
			found = true;
		}
	}

	// Compute insertion in the place of V
	TimeWindowData twData = MergeTWDataRecursive(V->prev->prefixTwData, U->twData, V->next->postfixTwData);
	int deltaCost = params->timeCost.get(V->prev->servedBy, U->servedBy) + params->timeCost.get(U->servedBy, V->next->servedBy) - params->timeCost.get(V->prev->servedBy, V->next->servedBy) + deltaPenaltyTimeWindows(twData, V->route->twData);
	if (!found || deltaCost < bestCost)
	{
		bestPosition = V->prev;
		bestCost = deltaCost;
	}

	return bestCost;
}

void LocalSearch::preprocessInsertions(Route* R1, Route* R2)
{
	for (Node* U = R1->depot->next; !U->isDepot; U = U->next)
	{
		// Performs the preprocessing
		U->deltaRemoval = params->timeCost.get(U->prev->servedBy, U->next->servedBy) - params->timeCost.get(U->prev->servedBy, U->servedBy) - params->timeCost.get(U->servedBy, U->next->servedBy);
		auto& currentOption = bestInsertClient[R2->cour][U->servedBy];
		if (R2->whenLastModified > currentOption.whenLastCalculated)
		{
			currentOption.reset();
			currentOption.whenLastCalculated = nbMoves;
			currentOption.bestCost[0] = params->timeCost.get(0, U->servedBy) + params->timeCost.get(U->servedBy, R2->depot->next->servedBy) - params->timeCost.get(0, R2->depot->next->servedBy);
			currentOption.bestLocation[0] = R2->depot;
			for (Node* V = R2->depot->next; !V->isDepot; V = V->next)
			{
				int deltaCost = params->timeCost.get(V->servedBy, U->servedBy) + params->timeCost.get(U->servedBy, V->next->servedBy) - params->timeCost.get(V->servedBy, V->next->servedBy);
				currentOption.compareAndAdd(deltaCost, V);
			}
		}
	}
}

void LocalSearch::preprocessInsertionsWithTW(Route* R1, Route* R2)
{
	TimeWindowData twData;
	for (Node* U = R1->depot->next; !U->isDepot; U = U->next)
	{
		// Performs the preprocessing
		// Note: when removing U and adding V to a route, the timewarp penalties may interact, however in most cases it will hold that
		// the reduced timewarp from removing U + added timewarp from adding V will be bigger than the actual delta timewarp such that assuming independence gives a conservative estimate

		if (R1->isDeltaRemovalTWOutdated)
		{
			twData = MergeTWDataRecursive(U->prev->prefixTwData, U->next->postfixTwData);
			U->deltaRemovalTW = params->timeCost.get(U->prev->servedBy, U->next->servedBy) - params->timeCost.get(U->prev->servedBy, U->servedBy) - params->timeCost.get(U->servedBy, U->next->servedBy) + deltaPenaltyTimeWindows(twData, R1->twData);
		}
		auto& currentOption = bestInsertClientTW[R2->cour][U->servedBy];
		if (R2->whenLastModified > currentOption.whenLastCalculated)
		{
			currentOption.reset();
			currentOption.whenLastCalculated = nbMoves;

			// Compute additional timewarp we get when inserting U in R2, this may be actually less if we remove U but we ignore this to have a conservative estimate
			twData = MergeTWDataRecursive(R2->depot->prefixTwData, U->twData, R2->depot->next->postfixTwData);

			currentOption.bestCost[0] = params->timeCost.get(0, U->servedBy) + params->timeCost.get(U->servedBy, R2->depot->next->servedBy) - params->timeCost.get(0, R2->depot->next->servedBy) + deltaPenaltyTimeWindows(twData, R2->twData);

			currentOption.bestLocation[0] = R2->depot;
			for (Node* V = R2->depot->next; !V->isDepot; V = V->next)
			{
				twData = MergeTWDataRecursive(V->prefixTwData, U->twData, V->next->postfixTwData);
				int deltaCost = params->timeCost.get(V->servedBy, U->servedBy) + params->timeCost.get(U->servedBy, V->next->servedBy) - params->timeCost.get(V->servedBy, V->next->servedBy) + deltaPenaltyTimeWindows(twData, R2->twData);
				currentOption.compareAndAdd(deltaCost, V);
			}
		}
	}
	R1->isDeltaRemovalTWOutdated = false;
}

TimeWindowData LocalSearch::getEdgeTwData(Node* U, Node* V)
{
	// TODO this could be cached?
	return MergeTWDataRecursive(U->twData, V->twData);
}

TimeWindowData LocalSearch::getRouteSegmentTwData(Node* U, Node* V)
{
	if (U->isDepot)
		return V->prefixTwData;
	if (V->isDepot)
		return U->postfixTwData;

	// Struct so this makes a copy
	TimeWindowData twData = U->twData;

	Node* mynode = U;
	const int targetPos = V->position;
	while (!(mynode == V))
	{
		if (mynode->isSeed && mynode->position + 4 <= targetPos)
		{
			twData = MergeTWDataRecursive(twData, mynode->toNextSeedTwD);
			mynode = mynode->nextSeed;
		}
		else
		{
			mynode = mynode->next;
			twData = MergeTWDataRecursive(twData, mynode->twData);
		}
	}
	return twData;
}

TimeWindowData LocalSearch::MergeTWDataRecursive(const TimeWindowData& twData1, const TimeWindowData& twData2)
{
	TimeWindowData mergedTwData;
	// Note, assume time equals cost
	int deltaCost = params->timeCost.get(twData1.lastNodeIndex, twData2.firstNodeIndex);
	int deltaDuration = deltaCost;
	int delta = twData1.duration - twData1.timeWarp + deltaDuration;
	int deltaWaitTime = std::max(twData2.earliestArrival - delta - twData1.latestArrival, 0);
	int deltaTimeWarp = std::max(twData1.earliestArrival + delta - twData2.latestArrival, 0);
	mergedTwData.firstNodeIndex = twData1.firstNodeIndex;
	mergedTwData.lastNodeIndex = twData2.lastNodeIndex;
	mergedTwData.duration = twData1.duration + twData2.duration + deltaDuration + deltaWaitTime;
	mergedTwData.timeWarp = twData1.timeWarp + twData2.timeWarp + deltaTimeWarp;
	mergedTwData.earliestArrival = std::max(twData2.earliestArrival - delta, twData1.earliestArrival) - deltaWaitTime;
	mergedTwData.latestArrival = std::min(twData2.latestArrival - delta, twData1.latestArrival) + deltaTimeWarp;
	mergedTwData.latestReleaseTime = std::max(twData1.latestReleaseTime, twData2.latestReleaseTime);
	return mergedTwData;
}

TimeWindowData LocalSearch::MergeTWDataRecursiveNewIndices(const TimeWindowData& twData1, const TimeWindowData& twData2, int twData1LastNodeIndex, int twData2FirstNodeIndex)
{
	TimeWindowData mergedTwData;
	// Note, assume time equals cost
	int deltaCost = params->timeCost.get(twData1LastNodeIndex, twData2FirstNodeIndex);
	int deltaDuration = deltaCost;
	int delta = twData1.duration - twData1.timeWarp + deltaDuration;
	int deltaWaitTime = std::max(twData2.earliestArrival - delta - twData1.latestArrival, 0);
	int deltaTimeWarp = std::max(twData1.earliestArrival + delta - twData2.latestArrival, 0);
	mergedTwData.firstNodeIndex = twData1.firstNodeIndex;
	mergedTwData.lastNodeIndex = twData2.lastNodeIndex;
	mergedTwData.duration = twData1.duration + twData2.duration + deltaDuration + deltaWaitTime;
	mergedTwData.timeWarp = twData1.timeWarp + twData2.timeWarp + deltaTimeWarp;
	mergedTwData.earliestArrival = std::max(twData2.earliestArrival - delta, twData1.earliestArrival) - deltaWaitTime;
	mergedTwData.latestArrival = std::min(twData2.latestArrival - delta, twData1.latestArrival) + deltaTimeWarp;
	mergedTwData.latestReleaseTime = std::max(twData1.latestReleaseTime, twData2.latestReleaseTime);
	return mergedTwData;
}

TimeWindowData LocalSearch::MergeTWDataRecursiveNewIndicesNewTW(const TimeWindowData& twData1, const TimeWindowData& twData2, int twData1LastNodeIndex, int twData2FirstNodeIndex, int earliestArrival1, int latestArrival1, int earliestArrival2, int latestArrival2)
{
	TimeWindowData mergedTwData;
	// Note, assume time equals cost
	int deltaCost = params->timeCost.get(twData1LastNodeIndex, twData2FirstNodeIndex);
	int deltaDuration = deltaCost;
	int delta = twData1.duration - twData1.timeWarp + deltaDuration;
	int deltaWaitTime = std::max(earliestArrival2- delta - earliestArrival1, 0);
	int deltaTimeWarp = std::max(earliestArrival1 + delta - latestArrival2, 0);
	mergedTwData.firstNodeIndex = twData1.firstNodeIndex;
	mergedTwData.lastNodeIndex = twData2.lastNodeIndex;
	mergedTwData.duration = twData1.duration + twData2.duration + deltaDuration + deltaWaitTime;
	mergedTwData.timeWarp = twData1.timeWarp + twData2.timeWarp + deltaTimeWarp;
	mergedTwData.earliestArrival = std::max(earliestArrival2 - delta, earliestArrival1) - deltaWaitTime;
	mergedTwData.latestArrival = std::min(latestArrival2 - delta, latestArrival1) + deltaTimeWarp;
	mergedTwData.latestReleaseTime = std::max(twData1.latestReleaseTime, twData2.latestReleaseTime);
	return mergedTwData;
}

TimeWindowData LocalSearch::MergeTWDataRecursiveNewIndicesNewTWNewParcelPoint(const TimeWindowData& twData1, const TimeWindowData& twData2, int twData1LastNodeIndex, int twData2FirstNodeIndex, int earliestArrival1, int latestArrival1, int earliestArrival2, int latestArrival2)
{
	TimeWindowData mergedTwData;
	// Note, assume time equals cost
	int deltaCost = params->timeCost.get(twData1LastNodeIndex, twData2FirstNodeIndex) + params->cli[params->nbClients+1].serviceDuration - params->cli[nodeU->cour].serviceDuration;
	int deltaDuration = deltaCost;
	int delta = twData1.duration - twData1.timeWarp + deltaDuration;
	int deltaWaitTime = std::max(earliestArrival2- delta - earliestArrival1, 0);
	int deltaTimeWarp = std::max(earliestArrival1 + delta - latestArrival2, 0);
	mergedTwData.firstNodeIndex = twData1.firstNodeIndex;
	mergedTwData.lastNodeIndex = twData2.lastNodeIndex;
	mergedTwData.duration = twData1.duration + twData2.duration + deltaDuration + deltaWaitTime;
	mergedTwData.timeWarp = twData1.timeWarp + twData2.timeWarp + deltaTimeWarp;
	mergedTwData.earliestArrival = std::max(earliestArrival2 - delta, earliestArrival1) - deltaWaitTime;
	mergedTwData.latestArrival = std::min(latestArrival2 - delta, latestArrival1) + deltaTimeWarp;
	mergedTwData.latestReleaseTime = std::max(twData1.latestReleaseTime, twData2.latestReleaseTime);
	return mergedTwData;
}

TimeWindowData LocalSearch::MergeTWDataRecursiveNewIndicesNewTWNoParcelPoint(const TimeWindowData& twData1, const TimeWindowData& twData2, int twData1LastNodeIndex, int twData2FirstNodeIndex, int earliestArrival1, int latestArrival1, int earliestArrival2, int latestArrival2)
{
	TimeWindowData mergedTwData;
	// Note, assume time equals cost
	int deltaCost = params->timeCost.get(twData1LastNodeIndex, twData2FirstNodeIndex) - params->cli[params->nbClients+1].serviceDuration + params->cli[nodeU->cour].serviceDuration;
	int deltaDuration = deltaCost;
	int delta = twData1.duration - twData1.timeWarp + deltaDuration;
	int deltaWaitTime = std::max(earliestArrival2- delta - earliestArrival1, 0);
	int deltaTimeWarp = std::max(earliestArrival1 + delta - latestArrival2, 0);
	mergedTwData.firstNodeIndex = twData1.firstNodeIndex;
	mergedTwData.lastNodeIndex = twData2.lastNodeIndex;
	mergedTwData.duration = twData1.duration + twData2.duration + deltaDuration + deltaWaitTime;
	mergedTwData.timeWarp = twData1.timeWarp + twData2.timeWarp + deltaTimeWarp;
	mergedTwData.earliestArrival = std::max(earliestArrival2 - delta, earliestArrival1) - deltaWaitTime;
	mergedTwData.latestArrival = std::min(latestArrival2 - delta, latestArrival1) + deltaTimeWarp;
	mergedTwData.latestReleaseTime = std::max(twData1.latestReleaseTime, twData2.latestReleaseTime);
	return mergedTwData;
}



void LocalSearch::moveMultipleClientsFromParcelPointToParcelPoint(std::vector < Node* > moveList, Node* fromP, Node* toP, Individual* I, int beforeOrAfter)
{
	// Set servedBy to parcelPoint, calculate discount and update load of parcel point
	int additionalLoad = 0;
	for (auto customer: moveList){
		if (toP->servedClients[customer->route->cour].size() > 0)
		{
			customer->next->prev = customer->prev;
			customer->prev->next = customer->next;
			customer->prev = toP->servedClients[customer->route->cour][toP->servedClients[customer->route->cour].size()-1];
			customer->next = toP->servedClients[customer->route->cour][toP->servedClients[customer->route->cour].size()-1]->next;
			toP->servedClients[customer->route->cour][toP->servedClients[customer->route->cour].size()-1]->next->prev = customer;
			toP->servedClients[customer->route->cour][toP->servedClients[customer->route->cour].size()-1]->next = customer;
		}

		customer->servedBy = toP->cour;
		customer->twData.firstNodeIndex = customer->servedBy;
		customer->twData.lastNodeIndex = customer->servedBy;
		RemoveCustomerFromServedClientsVector(customer->servedByParcelPoint->servedClients[customer->route->cour], customer);
		fromP->totalClientsServed -= 1;
		customer->servedByParcelPoint = toP;
		toP->servedClients[customer->route->cour].push_back(customer);
		toP->totalClientsServed += 1;
		I->servedByVec[customer->cour-1] = customer->servedBy;
		customer->discount = params->priceSensitivity;
		additionalLoad += params->cli[customer->cour].demand;
		
		if (beforeOrAfter == 1)
		{
			if (customer != nodeUPrev)
			{
				insertSingleNode(customer, nodeUPrev);
			}
		}else if (beforeOrAfter == 2){
			if (customer->servedBy == nodeX->servedBy)
			{
				insertSingleNode(customer, nodeX->prev);
			}else{
				if (nodeX->prev != customer){
					insertSingleNode(customer, nodeX->prev);
				}
	
			}
		}
	}

	toP->load = toP->load + additionalLoad;
	fromP->load = fromP->load - additionalLoad;

}


void LocalSearch::moveMultipleClientsToParcelPoint(std::vector < Node* > moveList, Node* P, Individual* I)
{

	int additionalLoad = 0;
	for (Node* customer: moveList){
		if (P->servedClients[customer->route->cour].size() > 0)
		{
			customer->next->prev = customer->prev;
			customer->prev->next = customer->next;
			customer->prev = P->servedClients[customer->route->cour][P->servedClients[customer->route->cour].size()-1];
			customer->next = P->servedClients[customer->route->cour][P->servedClients[customer->route->cour].size()-1]->next;
			P->servedClients[customer->route->cour][P->servedClients[customer->route->cour].size()-1]->next->prev = customer;
			P->servedClients[customer->route->cour][P->servedClients[customer->route->cour].size()-1]->next = customer;
			//insertSingleNode(customer, P->servedClients[routeU->cour][P->servedClients[routeU->cour].size()-1]);
		}

		if (customer->servedBy != customer->cour) RemoveCustomerFromServedClientsVector(P->servedClients[routeU->cour], customer);

		customer->servedBy = P->cour;
		customer->twData.firstNodeIndex = customer->servedBy;
		customer->twData.lastNodeIndex = customer->servedBy;
		customer->twData.duration = params->cli[customer->servedBy].serviceDuration;
		customer->twData.timeWarp = 0;
		customer->twData.earliestArrival = params->cli[customer->servedBy].earliestArrival;
		customer->twData.latestArrival = params->cli[customer->servedBy].latestArrival;
		P->servedClients[routeU->cour].push_back(customer);
		P->totalClientsServed += 1;
		customer->servedByParcelPoint = P;
		I->servedByVec[customer->cour-1] = customer->servedBy;
		customer->discount = params->priceSensitivity;
		additionalLoad += params->cli[customer->cour].demand; // Increment load
	}
	P->load = P->load + additionalLoad;

}

void:: LocalSearch::moveMultipleClientsBackToClients(std::vector < Node* > moveList, Node* P, Individual* I, int beforeOrAfter)
{
	int loadReduction = 0;
	for (auto customer: moveList){
		customer->servedBy = customer->cour;
		customer->twData.firstNodeIndex = customer->cour;
		customer->twData.lastNodeIndex = customer->cour;
		customer->twData.duration = params->cli[customer->cour].serviceDuration;
		customer->twData.earliestArrival = params->cli[customer->cour].earliestArrival;
		customer->twData.latestArrival = params->cli[customer->cour].latestArrival;
		RemoveCustomerFromServedClientsVector(P->servedClients[customer->route->cour], customer);
		P->totalClientsServed -= 1;
		customer->servedByParcelPoint = nullptr;
		I->servedByVec[customer->cour-1] = customer->cour;
		customer->discount = 0;
		loadReduction += params->cli[customer->cour].demand; // Increment load

		if (beforeOrAfter == 1)
		{
			if (customer != nodeUPrev)
			{
				insertSingleNode(customer, nodeUPrev);
			}
		}else if (beforeOrAfter == 2){
			if (customer->servedBy == nodeX->servedBy)
			{
				insertSingleNode(customer, nodeX->prev);
			}else{
				if (nodeX->prev != customer){
					insertSingleNode(customer, nodeX->prev);
				}
	
			}
		}

	}

	P->load = P->load - loadReduction;

}

void LocalSearch::insertNode(Node* toInsert, Node* insertionPoint)
{
	Node* firstToInsert = toInsert;
	int oldRouteCour = toInsert->route->cour;
	int newRouteCour = insertionPoint->route->cour;
	toInsert->route = insertionPoint->route;
	firstToInsert->route = insertionPoint->route;
	if (toInsert->servedBy != toInsert->cour && oldRouteCour != newRouteCour){
		RemoveCustomerFromServedClientsVector(toInsert->servedByParcelPoint->servedClients[oldRouteCour], toInsert);
		toInsert->servedByParcelPoint->servedClients[newRouteCour].push_back(toInsert);
	}

	if (toInsert->servedBy == toInsert->prev->servedBy){
		while (firstToInsert->servedBy == firstToInsert->prev->servedBy)
		{
			firstToInsert = firstToInsert->prev;
			firstToInsert->route = insertionPoint->route;
			if (oldRouteCour != newRouteCour){
				RemoveCustomerFromServedClientsVector(firstToInsert->servedByParcelPoint->servedClients[oldRouteCour], firstToInsert);
				firstToInsert->servedByParcelPoint->servedClients[newRouteCour].push_back(firstToInsert);
			}
		}
	};

	firstToInsert->prev->next = toInsert->next;
	toInsert->next->prev = firstToInsert->prev;
	insertionPoint->next->prev = toInsert;
	firstToInsert->prev = insertionPoint;
	toInsert->next = insertionPoint->next;
	insertionPoint->next = firstToInsert;
}

void LocalSearch::insertSingleNode(Node* toInsert, Node* insertionPoint)
{
	int oldRouteCour = toInsert->route->cour;
	int newRouteCour = insertionPoint->route->cour;
	if (toInsert->servedBy != toInsert->cour && oldRouteCour != newRouteCour){
		RemoveCustomerFromServedClientsVector(toInsert->servedByParcelPoint->servedClients[oldRouteCour], toInsert);
		toInsert->servedByParcelPoint->servedClients[newRouteCour].push_back(toInsert);
	}

	toInsert->prev->next = toInsert->next;
	toInsert->next->prev = toInsert->prev;
	insertionPoint->next->prev = toInsert;
	toInsert->prev = insertionPoint;
	toInsert->next = insertionPoint->next;
	insertionPoint->next = toInsert;
	toInsert->route = insertionPoint->route;
}


void LocalSearch::swapParcelPointClients(Node* U, Node* V, Individual* I)
{
	int routeCourU = U->route->cour;
	int routeCourV = V->route->cour;
	RemoveCustomerFromServedClientsVector(U->servedByParcelPoint->servedClients[routeCourU], U);
	RemoveCustomerFromServedClientsVector(V->servedByParcelPoint->servedClients[routeCourV], V);
	U->servedByParcelPoint->servedClients[routeCourU].push_back(V);
	V->servedByParcelPoint->servedClients[routeCourV].push_back(U);
	U->servedBy = V->servedByParcelPoint->cour;
	V->servedBy = U->servedByParcelPoint->cour;
	Node* oldServedByParcelPointU = U->servedByParcelPoint;
	Node* oldSservedByParcelPointV = V->servedByParcelPoint;
	U->servedByParcelPoint = oldSservedByParcelPointV;
	V->servedByParcelPoint = oldServedByParcelPointU;
	I->servedByVec[U->cour-1] = U->servedBy;
	I->servedByVec[V->cour-1] = V->servedBy;
	oldServedByParcelPointU->load += params->cli[V->cour].demand - params->cli[U->cour].demand; // Change load
	oldSservedByParcelPointV->load += params->cli[U->cour].demand - params->cli[V->cour].demand; // Change load
	U->discount = params->priceSensitivity;
	V->discount = params->priceSensitivity;
	nodeU->route->load += params->cli[V->cour].demand - params->cli[U->cour].demand;
	nodeV->route->load += params->cli[U->cour].demand - params->cli[V->cour].demand;

	Node* myVPred = V->prev;
	Node* myVSuiv = V->next;
	Node* myUPred = U->prev;
	Node* myUSuiv = U->next;
	Route* myRouteU = U->route;
	Route* myRouteV = V->route;

	myUPred->next = V;
	myUSuiv->prev = V;
	myVPred->next = U;
	myVSuiv->prev = U;

	U->prev = myVPred;
	U->next = myVSuiv;
	V->prev = myUPred;
	V->next = myUSuiv;

	U->route = myRouteV;
	V->route = myRouteU;
}

void LocalSearch::swapSingleNode(Node* U, Node* V)
{
	int routeCourU = U->route->cour;
	int routeCourV = V->route->cour;
	if (U->servedBy != U->cour && routeCourU != routeCourV){
		RemoveCustomerFromServedClientsVector(U->servedByParcelPoint->servedClients[routeCourU], U);
		U->servedByParcelPoint->servedClients[routeCourV].push_back(U);
		RemoveCustomerFromServedClientsVector(V->servedByParcelPoint->servedClients[routeCourV], V);
		V->servedByParcelPoint->servedClients[routeCourU].push_back(V);
	}

	Node* myVPred = V->prev;
	Node* myVSuiv = V->next;
	Node* myUPred = U->prev;
	Node* myUSuiv = U->next;
	Route* myRouteU = U->route;
	Route* myRouteV = V->route;

	myUPred->next = V;
	myUSuiv->prev = V;
	myVPred->next = U;
	myVSuiv->prev = U;

	U->prev = myVPred;
	U->next = myVSuiv;
	V->prev = myUPred;
	V->next = myUSuiv;

	U->route = myRouteV;
	V->route = myRouteU;
}


void LocalSearch::swapNode(Node* U, Node* V)
{
	int routeCourU = U->route->cour;
	int routeCourV = V->route->cour;

	if (U->servedBy != U->cour && routeCourU != routeCourV)
	{
		RemoveCustomerFromServedClientsVector(U->servedByParcelPoint->servedClients[routeCourU], U);
		U->servedByParcelPoint->servedClients[routeCourV].push_back(U);
	}
	if (V->servedBy != V->cour && routeCourU != routeCourV)
	{
		RemoveCustomerFromServedClientsVector(V->servedByParcelPoint->servedClients[routeCourV], V);
		V->servedByParcelPoint->servedClients[routeCourU].push_back(V);
	}

	Route* myRouteU = U->route;
	Route* myRouteV = V->route;

	Node* firstToInsertU = U;
	firstToInsertU->route = myRouteV;
	if (U->servedBy == U->prev->servedBy){
		while (firstToInsertU->servedBy == firstToInsertU->prev->servedBy)
		{
			firstToInsertU = firstToInsertU->prev;
			firstToInsertU->route = myRouteV;
			if (routeCourU != routeCourV)
			{
				RemoveCustomerFromServedClientsVector(firstToInsertU->servedByParcelPoint->servedClients[routeCourU], firstToInsertU);
				firstToInsertU->servedByParcelPoint->servedClients[routeCourV].push_back(firstToInsertU);
			}
		}
	};

	Node* firstToInsertV = V;
	firstToInsertV->route = myRouteU;
	if (V->servedBy == V->prev->servedBy){
		while (firstToInsertV->servedBy == firstToInsertV->prev->servedBy)
		{
			firstToInsertV = firstToInsertV->prev;
			firstToInsertV->route = myRouteU;
			if (routeCourU != routeCourV)
			{
				RemoveCustomerFromServedClientsVector(V->servedByParcelPoint->servedClients[routeCourV], firstToInsertV);
				firstToInsertV->servedByParcelPoint->servedClients[routeCourU].push_back(firstToInsertV);
			}
		}
	};

	Node* myVPred = firstToInsertV->prev;
	Node* myVSuiv = V->next;
	Node* myUPred = firstToInsertU->prev;
	Node* myUSuiv = U->next;

	myUPred->next = firstToInsertV;
	myUSuiv->prev = V;
	myVPred->next = firstToInsertU;
	myVSuiv->prev = U;

	firstToInsertU->prev = myVPred;
	U->next = myVSuiv;
	firstToInsertV->prev = myUPred;
	V->next = myUSuiv;

}

void LocalSearch::printRoute(Route* myRoute, bool servedBy)
{
	Node* mynode = myRoute->depot;
	bool firstIt = true;
	std::cout << "Hello:"; // Route IDs start at 1 in the file format
	while (!mynode->isDepot || firstIt)
	{
		firstIt = false;
		mynode = mynode->next;
		// Here we print the order of customers that we visit
		if (servedBy){
			std::cout << " " << mynode->servedBy;
		}else{
			std::cout << " " << mynode->cour;
		}
	}
	std::cout << std::endl;
}


void LocalSearch::printServedVector(Node*  P, int routeCour)
{
	std::cout << "Served Clients:";
	for (auto i: P->servedClients[routeCour]){
		std::cout << i->cour << ' ';
	}
	std::cout << std::endl;
}

void LocalSearch::updateRouteData(Route* myRoute)
{
	int myplace = 0;
	int myVehicleload = 0;
	int mytime = 0;
	int myReversalDistance = 0;
	int cumulatedX = 0;
	int cumulatedY = 0;

	Node* mynode = myRoute->depot;
	mynode->position = 0;
	mynode->cumulatedLoad = 0;
	mynode->cumulatedTime = 0;
	mynode->cumulatedReversalDistance = 0;

	bool firstIt = true;
	TimeWindowData seedTwD;
	Node* seedNode = nullptr;
	//printRoute(myRoute, true);
	while (!mynode->isDepot || firstIt)
	{
		mynode = mynode->next;
		myplace++;
		mynode->position = myplace;
		myVehicleload += params->cli[mynode->cour].demand;

		mytime += params->timeCost.get(mynode->prev->servedBy, mynode->servedBy);
		
		if (mynode->prev->servedBy != mynode->servedBy){
			mytime += params->cli[mynode->servedBy].serviceDuration;
		};

		myReversalDistance += params->timeCost.get(mynode->servedBy, mynode->prev->servedBy) - params->timeCost.get(mynode->prev->servedBy, mynode->servedBy);
		mynode->cumulatedLoad = myVehicleload;
		mynode->cumulatedTime = mytime;
		mynode->cumulatedReversalDistance = myReversalDistance;

		if (mynode->servedBy != mynode->cour){
			mynode->twData.timeWarp = 0;
		}
		//std::cout<<mynode->twData.latestArrival<<std::endl;
		if ((mynode->prev->servedBy != mynode->servedBy) || firstIt){
			mynode->prefixTwData = MergeTWDataRecursive(mynode->prev->prefixTwData, mynode->twData);
		}else{
			mynode->twData = mynode->prev->twData;
			mynode->prefixTwData = mynode->prev->prefixTwData;
		};
		
		firstIt = false;

		mynode->isSeed = false;
		mynode->nextSeed = nullptr;
		if (!mynode->isDepot)
		{
			cumulatedX += params->cli[mynode->cour].coordX;
			cumulatedY += params->cli[mynode->cour].coordY;
			if (firstIt) myRoute->sector.initialize(params->cli[mynode->cour].polarAngle);
			else myRoute->sector.extend(params->cli[mynode->cour].polarAngle);
			if (myplace % 4 == 0)
			{
				if (seedNode != nullptr)
				{
					seedNode->isSeed = true;
					seedNode->toNextSeedTwD = MergeTWDataRecursive(seedTwD, mynode->twData);
					seedNode->nextSeed = mynode;
				}
				seedNode = mynode;
			}
			else if (myplace % 4 == 1)
			{
				seedTwD = mynode->twData;
			}
			else
			{
				seedTwD = MergeTWDataRecursive(seedTwD, mynode->twData);
			}
		}
		
	}
	myRoute->duration = mytime; // Driving duration + service duration, excluding waiting time / time warp
	myRoute->load = myVehicleload;
	myRoute->twData = mynode->prefixTwData;
	myRoute->penalty = penaltyExcessLoadVehicle(myVehicleload) + penaltyTimeWindows(myRoute->twData);
	myRoute->nbCustomers = myplace - 1;
	myRoute->reversalDistance = myReversalDistance;
	// Remember "when" this route has been last modified (will be used to filter unnecessary move evaluations)
	myRoute->whenLastModified = nbMoves;
	myRoute->isDeltaRemovalTWOutdated = true;
	// Time window data in reverse direction, mynode should be end depot now
	firstIt = true;
	while (!mynode->isDepot || firstIt)
	{
		firstIt = false;
		mynode = mynode->prev;
		if (mynode->next->servedBy != mynode->servedBy){
			mynode->postfixTwData = MergeTWDataRecursive(mynode->twData, mynode->next->postfixTwData);
		}else{
			mynode->postfixTwData = mynode->next->postfixTwData;
		};
	}

	if (myRoute->nbCustomers == 0)
	{
		myRoute->polarAngleBarycenter = 1.e30;
		emptyRoutes.insert(myRoute->cour);
	}
	else
	{
		myRoute->polarAngleBarycenter = atan2(cumulatedY / static_cast<double>(myRoute->nbCustomers) - params->cli[0].coordY, cumulatedX / static_cast<double>(myRoute->nbCustomers) - params->cli[0].coordX);
		// Enforce minimum size of circle sector
		if (params->minCircleSectorSize > 0)
		{
			const int growSectorBy = (params->minCircleSectorSize - myRoute->sector.positive_mod(myRoute->sector.end - myRoute->sector.start) + 1) / 2;
			if (growSectorBy > 0)
			{
				myRoute->sector.extend(myRoute->sector.start - growSectorBy);
				myRoute->sector.extend(myRoute->sector.end + growSectorBy);
			}
		}
		emptyRoutes.erase(myRoute->cour);
	}
}

CostSol LocalSearch::getCostSol(bool usePenaltiesLS)
{
	CostSol myCostSol;

	myCostSol.nbRoutes = 0; // TODO
	std::vector<int> parcelPointsLoad(params->nbParcelPoints, 0);
	for (int r = 0; r < params->nbVehicles; r++)
	{
		Route* myRoute = &routes[r];
		//myCostSol.distance += myRoute->duration;
		myCostSol.vehicleCapacityExcess += std::max(0, myRoute->load - params->vehicleCapacity);
		myCostSol.waitTime += 0; // TODO. Currently not penalized
		myCostSol.timeWarp += myRoute->twData.timeWarp;
		// Check if parcel point is employed
		bool firstIt = true;
		Node* mynode = myRoute->depot;
		while (!mynode->isDepot || firstIt)
		{
			myCostSol.distance += params->timeCost.get(mynode->servedBy, mynode->next->servedBy);
			mynode = mynode->next;
			if (mynode->cour != mynode->servedBy){
				myCostSol.discountGiven += mynode->discount;
				parcelPointsLoad[mynode->servedBy-params->nbClients-1] += params->cli[mynode->cour].demand;
			}
			firstIt = false;
		}
	}

	int counter = 0;
	for(int i : parcelPointsLoad){
		if (i > params->cellsPerParcelPoint[counter]){
			myCostSol.parcelPointCapacityExcess += i - params->cellsPerParcelPoint[counter];
		}
		counter += 1;
	}

	// Subtract service durations which are not part of distance

	if (usePenaltiesLS)
	{
		myCostSol.penalizedCost = myCostSol.distance + myCostSol.discountGiven + myCostSol.vehicleCapacityExcess * penaltyVehicleCapacityLS + myCostSol.timeWarp * penaltyTimeWarpLS + myCostSol.parcelPointCapacityExcess * penaltyParcelPointCapacityLS;
	}
	else
	{
		myCostSol.penalizedCost = myCostSol.distance + myCostSol.discountGiven + myCostSol.vehicleCapacityExcess * params->penaltyVehicleCapacity + myCostSol.timeWarp * params->penaltyTimeWarp + myCostSol.parcelPointCapacityExcess * penaltyParcelPointCapacityLS;
	}
	return myCostSol;
}

void LocalSearch::loadIndividual(Individual* indiv)
{
	emptyRoutes.clear();
	shuffleClientOrderInPP = false;
	nbMoves = 0;
	TimeWindowData depotTwData;
	depotTwData.firstNodeIndex = 0;
	depotTwData.lastNodeIndex = 0;
	depotTwData.duration = 0;
	depotTwData.timeWarp = 0;
	depotTwData.earliestArrival = params->cli[0].earliestArrival;
	depotTwData.latestArrival = params->cli[0].latestArrival;
	depotTwData.latestReleaseTime = params->cli[0].releaseTime;

	// Initializing time window data (before loop since it is needed in update route)
	for (int i = 1; i <= params->nbClients; i++)
	{
		TimeWindowData* myTwData = &clients[i].twData;
		myTwData->firstNodeIndex = indiv->servedByVec[i-1];
		myTwData->lastNodeIndex = indiv->servedByVec[i-1];
		myTwData->duration = params->cli[i].serviceDuration;
		myTwData->earliestArrival = params->cli[i].earliestArrival;
		myTwData->latestArrival = params->cli[i].latestArrival;
		myTwData->latestReleaseTime = params->cli[i].releaseTime;
		// myTwData->load = params->cli[i].demand;
	}

	for (int i=0; i<params->nbParcelPoints; i++){
		parcelPoints[i].load = 0;
		parcelPoints[i].totalClientsServed = 0;
		for (int j=0; j<params->nbVehicles; j++)
		{
			parcelPoints[i].servedClients[j].clear();
		}
	}

	parcelPointsPointer.clear();
	// Initializing parcel points pointers
	for (int j=0; j<params->nbParcelPoints; j++){
		Node* pp = &parcelPoints[j];
		for (auto j: pp->servedClients){
			j.clear();
		}
		parcelPointsPointer.push_back(pp);
	}

	for (int r = 0; r < params->nbVehicles; r++)
	{
		Node* myDepot = &depots[r];
		Node* myDepotFin = &depotsEnd[r];
		Route* myRoute = &routes[r];
		std::vector<Node*> nodesNotInRightOrder;
		std::vector<bool> wrongOrder(params->nbParcelPoints, false);
		myDepot->prev = myDepotFin;
		myDepotFin->next = myDepot;
		if (!indiv->chromR[r].empty())
		{
			Node* myClient = indiv->chromR[r][0];
			myClient->servedBy = indiv->servedByVec[myClient->cour-1];
			myClient->route = myRoute;
			myClient->prev = myDepot;
			myDepot->next = myClient;
			myClient->servedByParcelPoint = nullptr;
			myClient->discount = 0;
			// Check if parcel point is employed and add demand to load if so
			if (myClient->cour != myClient->servedBy){
				int clientServedBy = myClient->servedBy;
				auto match = std::find_if(parcelPointsPointer.begin(), parcelPointsPointer.end(), [clientServedBy] (const Node* o) -> bool {return o->cour == clientServedBy;});
				myClient->servedByParcelPoint = match[0];
				myClient->discount = params->priceSensitivity;
				myClient->twData.duration = params->cli[myClient->servedBy].serviceDuration;
				myClient->twData.earliestArrival = params->cli[0].earliestArrival;
				myClient->twData.latestArrival = params->cli[0].latestArrival;
				myClient->servedByParcelPoint->load += params->cli[myClient->cour].demand;
				myClient->servedByParcelPoint->totalClientsServed += 1;
				myClient->servedByParcelPoint->servedClients[r].push_back(myClient);
			}

			for (int i = 1; i < static_cast<int>(indiv->chromR[r].size()); i++)
			{
				Node* myClientPred = myClient;
				myClient = indiv->chromR[r][i];
				myClient->servedBy = indiv->servedByVec[myClient->cour-1];
				myClient->prev = myClientPred;
				myClientPred->next = myClient;
				myClient->route = myRoute;
				myClient->servedByParcelPoint = nullptr;
				myClient->discount = 0;
				// Check if parcel point is employed and add demand to load if so
				if (myClient->cour != myClient->servedBy){
					int clientServedBy = myClient->servedBy;
					auto match = std::find_if(parcelPointsPointer.begin(), parcelPointsPointer.end(), [clientServedBy] (const Node* o) -> bool {return o->cour == clientServedBy;});
					myClient->servedByParcelPoint = match[0];
					myClient->discount = params->priceSensitivity;
					myClient->twData.duration = params->cli[myClient->servedBy].serviceDuration;
					myClient->twData.earliestArrival = params->cli[0].earliestArrival;
					myClient->twData.latestArrival = params->cli[0].latestArrival;
					myClient->servedByParcelPoint->load += params->cli[myClient->cour].demand;
					if((myClient->prev->servedBy != myClient->servedBy && myClient->servedByParcelPoint->servedClients[myRoute->cour].size() > (size_t) 0 )|| wrongOrder[myClient->servedBy-params->nbClients-1]){
						nodesNotInRightOrder.push_back(myClient);
						shuffleClientOrderInPP = true;
						wrongOrder[myClient->servedBy-params->nbClients-1] = true;
					}
					myClient->servedByParcelPoint->totalClientsServed += 1;
					myClient->servedByParcelPoint->servedClients[myRoute->cour].push_back(myClient);
					
				}
			}
			myClient->next = myDepotFin;
			myDepotFin->prev = myClient;
		}
		else
		{
			myDepot->next = myDepotFin;
			myDepotFin->prev = myDepot;
		}

		myDepot->twData = depotTwData;
		myDepot->prefixTwData = depotTwData;
		myDepot->postfixTwData = depotTwData;
		myDepot->isSeed = false;

		myDepotFin->twData = depotTwData;
		myDepotFin->prefixTwData = depotTwData;
		myDepotFin->postfixTwData = depotTwData;
		myDepotFin->isSeed = false;


		for (auto node: nodesNotInRightOrder){
			insertSingleNode(node,node->servedByParcelPoint->servedClients[node->route->cour][0]);
		}


		updateRouteData(&routes[r]);
		routes[r].whenLastTestedLargeNb = -1;
		bestInsertInitializedForRoute[r] = false;
	}
	// Shuffle vector to diversify the search
	std::shuffle(parcelPointsPointer.begin(), parcelPointsPointer.end(), params->rng);


	for (int i = 1; i <= params->nbClients; i++) // Initializing memory structures
		clients[i].whenLastTestedRI = -1;

}

void LocalSearch::exportIndividual(Individual* indiv)
{
	std::vector < std::pair <double, int> > routePolarAngles;
	for (int r = 0; r < params->nbVehicles; r++)
		routePolarAngles.push_back(std::pair <double, int>(routes[r].polarAngleBarycenter, r));
	std::sort(routePolarAngles.begin(), routePolarAngles.end()); // empty routes have a polar angle of 1.e30, and therefore will always appear at the end

	int pos = 0;
	for (int r = 0; r < params->nbVehicles; r++)
	{
		indiv->chromR[r].clear();
		Node* node = depots[routePolarAngles[r].second].next;
		while (!node->isDepot)
		{
			indiv->chromT[pos] = node;
			indiv->chromR[r].push_back(node);
			node = node->next;
			pos++;
		}
	}

	indiv->evaluateCompleteCost();
}

LocalSearch::LocalSearch(Params* params) : params(params)
{
	clients = std::vector < Node >(params->nbClients + 1);
	routes = std::vector < Route >(params->nbVehicles);
	depots = std::vector < Node >(params->nbVehicles);
	depotsEnd = std::vector < Node >(params->nbVehicles);
	bestInsertInitializedForRoute = std::vector < bool >(params->nbVehicles, false);
	bestInsertClient = std::vector < std::vector <ThreeBestInsert> >(params->nbVehicles, std::vector <ThreeBestInsert>(params->nbClients + params->nbParcelPoints + 1));
	bestInsertClientTW = std::vector < std::vector <ThreeBestInsert> >(params->nbVehicles, std::vector <ThreeBestInsert>(params->nbClients + params->nbParcelPoints + 1));

	parcelPoints = std::vector < Node >(params->nbParcelPoints);
	for (int i = 0; i < params->nbParcelPoints; i++)
	{
		parcelPoints[i].cour = i + params->nbClients + 1;
		parcelPoints[i].load = 0;
		parcelPoints[i].isDepot = false;
		parcelPoints[i].isParcelPoint = true;
		parcelPoints[i].servedClients = std::vector<std::vector<Node*>>(params->nbVehicles);
		parcelPoints[i].totalClientsServed = 0;
	}

	for (int i = 0; i <= params->nbClients; i++)
	{
		clients[i].cour = i;
		clients[i].servedBy = i;
		clients[i].isDepot = false;
	}

	for (int i = 0; i < params->nbVehicles; i++)
	{
		routes[i].cour = i;
		routes[i].depot = &depots[i];
		depots[i].cour = 0;
		depots[i].isDepot = true;
		depots[i].route = &routes[i];
		depotsEnd[i].cour = 0;
		depotsEnd[i].isDepot = true;
		depotsEnd[i].route = &routes[i];
	}
	for (int i = 1; i <= params->nbClients; i++) orderNodes.push_back(i);
	for (int r = 0; r < params->nbVehicles; r++) orderRoutes.push_back(r);
}
