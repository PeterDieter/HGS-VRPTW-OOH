#include <algorithm>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <numeric>

#include "Individual.h"
#include "Params.h"
#include "LocalSearch.h"


void Individual::evaluateCompleteCost()
{
	// Create an object to store all information regarding solution costs
	myCostSol = CostSol();
	int vehiclesUsed = 0;


	// Initialize servedBy with servedByVector
	for (auto veh : chromR){
		for (auto node : veh){
			node->servedBy = servedByVec[node->cour-1];
		};
	};

	// Loop over all routes that are not empty
	std::vector<int> parcelPointsLoad(params->nbParcelPoints, 0);
	for (int r = 0; r < params->nbVehicles; r++)
	{
		if (!chromR[r].empty())
		{
			vehiclesUsed += 1;
			int latestReleaseTime = params->cli[chromR[r][0]->cour].releaseTime;
			for (int i = 1; i < static_cast<int>(chromR[r].size()); i++)
			{
				latestReleaseTime = std::max(latestReleaseTime, params->cli[chromR[r][i]->cour].releaseTime);
			}
			// Get the distance, load, serviceDuration and time associated with the vehicle traveling from the depot to the first client
			// Assume depot has service time 0 and earliestArrival 0
			int distance = params->timeCost.get(0, chromR[r][0]->servedBy);
			int load = params->cli[chromR[r][0]->cour].demand;
			int service = params->cli[chromR[r][0]->servedBy].serviceDuration;

			// Check if parcel point is employed
			if (chromR[r][0]->cour != chromR[r][0]->servedBy){
				myCostSol.discountGiven += chromR[r][0]->discount;
				parcelPointsLoad[chromR[r][0]->servedBy-params->nbClients-1] += 1;
			}

			// Running time excludes service of current node. This is the time that runs with the vehicle traveling
			// We start the route at the latest release time (or later but then we can just wait and there is no penalty for waiting)
			int time = latestReleaseTime + distance;
			int waitTime = 0;
			int timeWarp = 0;
			// Add possible waiting time
			if (time < params->cli[chromR[r][0]->servedBy].earliestArrival)
			{
				// Don't add wait time since we can start route later 
				// (doesn't really matter since there is no penalty anyway)
				// waitTime += params->cli[chromR[r][0]].earliestArrival - time;
				time = params->cli[chromR[r][0]->servedBy].earliestArrival;
			}
			// Add possible time warp
			else if (time > params->cli[chromR[r][0]->servedBy].latestArrival)
			{
				if (chromR[r][0]->cour != chromR[r][0]->servedBy){
					timeWarp += 0;
				}else{
					timeWarp += time - params->cli[chromR[r][0]->servedBy].latestArrival;
					time = params->cli[chromR[r][0]->servedBy].latestArrival;
				}
			}
			predecessors[chromR[r][0]->cour] = 0;

			// Loop over all clients for this vehicle
			for (int i = 1; i < static_cast<int>(chromR[r].size()); i++)
			{
				// Sum the distance, load, serviceDuration and time associated with the vehicle traveling from the depot to the next client
				distance += params->timeCost.get(chromR[r][i - 1]->servedBy, chromR[r][i]->servedBy);
				load += params->cli[chromR[r][i]->cour].demand;
				service += params->cli[chromR[r][i]->cour].serviceDuration;
				// Check if parcel point is employed
				if (chromR[r][i]->cour != chromR[r][i]->servedBy){
					myCostSol.discountGiven += chromR[r][i]->discount;
					parcelPointsLoad[chromR[r][i]->servedBy-params->nbClients-1] += 1;
				}
				if (chromR[r][i-1]->cour != chromR[r][i-1]->servedBy){
					if (chromR[r][i]->servedBy != chromR[r][i-1]->servedBy){
						time = time + params->cli[chromR[r][i - 1]->servedBy].serviceDuration + params->timeCost.get(chromR[r][i - 1]->servedBy, chromR[r][i]->servedBy);
					};
				}else{
					time = time + params->cli[chromR[r][i - 1]->cour].serviceDuration + params->timeCost.get(chromR[r][i - 1]->servedBy, chromR[r][i]->servedBy);
				};
				

				// Add possible waiting time
				if (time < params->cli[chromR[r][i]->servedBy].earliestArrival)
				{
					waitTime += params->cli[chromR[r][i]->servedBy].earliestArrival - time;
					time = params->cli[chromR[r][i]->servedBy].earliestArrival;
				}
				// Add possible time warp
				else if (time > params->cli[chromR[r][i]->servedBy].latestArrival)
				{
					if (chromR[r][i]->cour != chromR[r][i]->servedBy){
						timeWarp += 0;
					}else{
						timeWarp += time - params->cli[chromR[r][i]->servedBy].latestArrival;
						time = params->cli[chromR[r][i]->servedBy].latestArrival;
					}
					
				}

				// Update predecessors and successors
				predecessors[chromR[r][i]->cour] = chromR[r][i - 1]->cour;
				successors[chromR[r][i - 1]->cour] = chromR[r][i]->cour;
				//servedByVec[chromR[r][i]->cour-1] = chromR[r][i]->servedBy;
			}

			// For the last client, the successors is the depot. Also update the distance and time
			successors[chromR[r][chromR[r].size() - 1]->cour] = 0;
			distance += params->timeCost.get(chromR[r][chromR[r].size() - 1]->servedBy, 0);
			
			int st = 0;
			if (chromR[r][chromR[r].size()-1]->servedBy != chromR[r][chromR[r].size()-1]->cour){
				if (chromR[r].size() > 1){
					if (chromR[r][chromR[r].size()-1]->servedBy != chromR[r][chromR[r].size()-2]->servedBy){
						st = params->cli[chromR[r][chromR[r].size() - 1]->servedBy].serviceDuration;
					}
				}
			}else{
				st = params->cli[chromR[r][chromR[r].size() - 1]->servedBy].serviceDuration;
			};

			time = time + st + params->timeCost.get(chromR[r][chromR[r].size() - 1]->servedBy, 0);
			
			// For the depot, we only need to check the end of the time window (add possible time warp)
			if (time > params->cli[0].latestArrival)
			{
				timeWarp += time - params->cli[0].latestArrival;
				time = params->cli[0].latestArrival;
			}
			// Update variables that track stats on the whole solution (all vehicles combined)
			myCostSol.distance += distance;
			myCostSol.waitTime += waitTime;
			myCostSol.timeWarp += timeWarp;
			myCostSol.nbRoutes++;
			if (load > params->vehicleCapacity)
			{
				myCostSol.vehicleCapacityExcess += load - params->vehicleCapacity;
			}
		}
	}
	
	int counter = 0;
	for(int i : parcelPointsLoad){
		if (i > params->cellsPerParcelPoint[counter]){
			myCostSol.parcelPointCapacityExcess += i - params->cellsPerParcelPoint[counter];
		}
		counter += 1;
	}

	// When all vehicles are dealt with, calculated total penalized cost and check if the solution is feasible. (Wait time does not affect feasibility)
	myCostSol.penalizedCost = vehiclesUsed*params->costPerVehicle + myCostSol.distance + myCostSol.discountGiven + myCostSol.vehicleCapacityExcess * params->penaltyVehicleCapacity + myCostSol.timeWarp * params->penaltyTimeWarp + myCostSol.waitTime * params->penaltyWaitTime + myCostSol.parcelPointCapacityExcess*params->penaltyParcelCapacity;
	isFeasible = (myCostSol.vehicleCapacityExcess < MY_EPSILON && myCostSol.timeWarp < MY_EPSILON && myCostSol.parcelPointCapacityExcess < MY_EPSILON);
	
}

void Individual::shuffleChromT()
{
	// Initialize the chromT with values from 1 to nbClients
	int counter = 0;
	for (int r = 0; r < params->nbVehicles; r++){
		if (!chromR[r].empty()){
			for (int i = 1; i < static_cast<int>(chromR[r].size()); i++) {
				chromT[counter] = chromR[r][i];
				counter += 1;
			}
		}
	}
	
	// ORIGINAL CODE:
	// for (int i = 0; i < params->nbClients; i++)
	// {
	// 	chromT[i] = i + 1;
	// }
	// Do a random shuffle chromT from begin to end
	std::shuffle(chromT.begin(), chromT.end(), params->rng);
}

void Individual::shuffleChromTConstruction()
{
	// Initialize the chromT with values from 1 to nbClients
	for (int i = 0; i < params->nbClients; i++){ 
		chromT[i] = new Node {.cour=i+1, .servedBy=i+1};
	}
	std::shuffle(chromT.begin(), chromT.end(), params->rng);
}

void Individual::removeProximity(Individual* indiv)
{
	// Get the first individual in indivsPerProximity
	auto it = indivsPerProximity.begin();
	// Loop over all individuals in indivsPerProximity until indiv is found
	while (it->second != indiv)
	{
		++it;
	}
	// Remove indiv from indivsPerProximity
	indivsPerProximity.erase(it);
}

double Individual::brokenPairsDistance(Individual* indiv2)
{
	// Initialize the difference to zero. Then loop over all clients of this individual
	int differences = 0;
	int differencesPP = 0;
	for (int j = 1; j <= params->nbClients; j++)
	{
		// Increase the difference if the successor of j in this individual is not directly linked to j in indiv2
		if (successors[j] != indiv2->successors[j] && successors[j] != indiv2->predecessors[j])
		{
			differences++;
		}

		if (servedByVec[j] != indiv2->servedByVec[j]){
			differencesPP++;
		}
		// Last loop covers all but the first arc. Increase the difference if the predecessor of j in this individual is not directly linked to j in indiv2
		if (predecessors[j] == 0 && indiv2->predecessors[j] != 0 && indiv2->successors[j] != 0)
		{
			differences++;
		}
	}
	return static_cast<double>(differences*(1-params->config.fractionOfParcelPointSimilarity)+differencesPP*(params->config.fractionOfParcelPointSimilarity)) / params->nbClients;
}

double Individual::averageBrokenPairsDistanceClosest(int nbClosest)
{
	double result = 0;
	int maxSize = std::min(nbClosest, static_cast<int>(indivsPerProximity.size()));
	auto it = indivsPerProximity.begin();
	for (int i = 0; i < maxSize; i++)
	{
		result += it->first;
		++it;
	}
	return result / maxSize;
}

void Individual::exportCVRPLibFormat(std::string fileName)
{
	std::cout << "----- WRITING SOLUTION WITH VALUE " << myCostSol.penalizedCost << " IN : " << fileName << std::endl;
	std::ofstream myfile(fileName);
	if (myfile.is_open())
	{
		for (int k = 0; k < params->nbVehicles; k++)
		{
			if (!chromR[k].empty())
			{
				// Here we print the order of customers that we visit 
				myfile << "Route #" << k + 1 << ":"; // Route IDs start at 1 in the file format
				for (auto node : chromR[k])
				{
					myfile << " " << node->cour;
				}
				myfile << std::endl;

				// Here we print the route of actual nodes that we drive to
				myfile << "ServedBy Route #" << k + 1 << ":"; // Route IDs start at 1 in the file format
				for (auto node : chromR[k])
				{
					myfile << " " << node->servedBy;
				}
				myfile << std::endl;

				// Here we print the discount given to each customer
				myfile << "Discount Route #" << k + 1 << ":"; // Route IDs start at 1 in the file format
				for (auto node : chromR[k])
				{
					myfile << " " << node->discount;
				}
				myfile << std::endl;
			}

		}
		myfile << "Cost " << myCostSol.penalizedCost << std::endl;
		myfile << "Time " << params->getTimeElapsedSeconds() << std::endl;
	}
	else std::cout << "----- IMPOSSIBLE TO OPEN: " << fileName << std::endl;
}

void Individual::printCVRPLibFormat()
{
	std::cout << "----- PRINTING SOLUTION WITH VALUE " << myCostSol.penalizedCost << std::endl;
	for (int k = 0; k < params->nbVehicles; k++)
	{
		if (!chromR[k].empty())
		{
			// Here we print the order of customers that we visit 
			std::cout << "Route #" << k + 1 << ":"; // Route IDs start at 1 in the file format
			for (auto node : chromR[k])
			{
				std::cout << " " << node->cour;
			}
			std::cout << std::endl;
			
			// Here we print the route of actual nodes that we drive to
			std::cout << "ServedBy Route #" << k + 1 << ":"; // Route IDs start at 1 in the file format
			for (auto node : chromR[k])
			{
				std::cout << " " << node->servedBy;
			}
			std::cout << std::endl;

			// Here we print the discount given to each customer
			std::cout << "Discount Route #" << k + 1 << ":"; // Route IDs start at 1 in the file format
			for (auto node : chromR[k])
			{
				std::cout << " " << node->discount;
			}
			std::cout << std::endl;
		}
		
	}
	std::cout << "Cost " << myCostSol.penalizedCost << std::endl;
	std::cout << "Time " << params->getTimeElapsedSeconds() << std::endl;
	fflush(stdout);
}

bool Individual::readCVRPLibFormat(std::string fileName, std::vector<std::vector<int>>& readSolution, double& readCost)
{
	readSolution.clear();
	std::ifstream inputFile(fileName);
	if (inputFile.is_open())
	{
		std::string inputString;
		inputFile >> inputString;
		// Loops as long as the first line keyword is "Route"
		for (int r = 0; inputString == "Route"; r++)
		{
			readSolution.push_back(std::vector<int>());
			inputFile >> inputString;
			getline(inputFile, inputString);
			std::stringstream ss(inputString);
			int inputCustomer;
			// Loops as long as there is an integer to read
			while (ss >> inputCustomer)
			{
				readSolution[r].push_back(inputCustomer);
			}
			inputFile >> inputString;
		}
		if (inputString == "Cost")
		{
			inputFile >> readCost;
			return true;
		}
		else std::cout << "----- UNEXPECTED WORD IN SOLUTION FORMAT: " << inputString << std::endl;
	}
	else std::cout << "----- IMPOSSIBLE TO OPEN: " << fileName << std::endl;
	return false;
}

Individual::Individual(Params* params, bool initializeChromTAndShuffle, bool initializeServedBy, bool constructingRandomSolution) : params(params), isFeasible(false), biasedFitness(0)
{
	successors = std::vector<int>(params->nbClients + 1);
	predecessors = std::vector<int>(params->nbClients + 1);
	servedByVec = std::vector<int>(params->nbClients);
	chromR = std::vector<std::vector<Node*>>(params->nbVehicles);
	chromT = std::vector<Node*>(params->nbClients);

	// Initialize servedBy vector with integer values from 1 to nbclients, i.e. only home deliveries
	if (initializeServedBy){
		std::iota(servedByVec.begin(), servedByVec.end(), 1);
	};

	if (initializeChromTAndShuffle)
	{
		if (constructingRandomSolution){
			shuffleChromTConstruction();
		} else{
			shuffleChromT();
		}
		
		
	}
}

Individual::Individual(): params(nullptr), isFeasible(false), biasedFitness(0)
{
	myCostSol.penalizedCost = 1.e30;
}
