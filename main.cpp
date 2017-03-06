/* 
   Simulation of Depth-Based Routing (DBR) for Underwater Sensor Networks
   @YSLin. May, 2016.
*/
//#pragma comment(linker, "/STACK:36777216")
//#pragma comment(linker, "/HEAP:36777216")

#pragma warning(disable:4996)

#include "main.h"

void main()
{
    srand(0); // initialize random seed
              // parameter = 0, 1, 2, ... or time(NULL)
    
    initialMessageAndInsert();          
    initializeNodePosition();
    initializeParameters();             
    printToCheckNeighborNodes();       
	printToCheckChannelModelValue();   

    generateInitialDepartureEventsForSource();

    while(currentTime < executionTime) {
        
        int nEvents = getNumberOfEventsInTimeslot(currentTime);  // nEvents: �b��timeslot�̦��X��events
        fprintf(tFile2, "before: (number of events, time slot): (%d, %d)\n", nEvents, currentTime);
        
        int nCollisions = 0;                                     
        if(collisionEnabled) {
            nCollisions = removeCollisionEvents(nEvents);        // �b��timeslot��collision�P�_�P�B�z
        }
        totalCollisions += nCollisions;      
        fprintf(tFile2, "mid: (number of collisions, total collisions, time slot): (%d, %d, %d)\n", 
			    nCollisions, totalCollisions, currentTime);
        
		// ����collision events��A���s�A���@����timeslot�̦��X��events
        nEvents = getNumberOfEventsInTimeslot(currentTime);
        fprintf(tFile2, "after: (number of events, time slot): (%d, %d)\n", nEvents, currentTime);
        
        collisionPrinting(nCollisions);        // �ˬd collisions (just for debugging)

        for(int i=0; i<nEvents; i++) {
            switch(eventQueue[i].eventType) {
                case event_ARRIVAL:
                    dataPacketProcessing(i);     
                    break;
                case event_DEPARTURE:
                    departurePacketProcessing(i);      
                    break;
                default:
                    break;
            }
        }

        removeEvents(nEvents); // �B�z��arrival�Mdeparture����remove events

		printNodeEnergyCurrentTimeslot(currentTime);   
		printCenterTableCurrentTimeslot(currentTime);  

        idleEnergyConsumption();                       
        checkAndGenerateNewPacket();                   
        checkNodePositionUpdate();                     
      
        currentTime += timeslotLength;
    }

    calculateStatistics();

    endingMessage();  
    closeLogFiles();  

    system("pause");                                   // �Ȱ��e����
}

void initialMessageAndInsert()
{
	printf("WitLab: Simulation of Depth-Based Routing (DBR) for Underwater Sensor Networks\n");
	printf("Number of nodes (source, sink, relay) = (%d, %d, %d)\n", numOfSourceNodes, numOfSinkNodes, numOfRelayNodes);

	// Dimensions: �i�����������Ŷ�size
	printf("Dimensions = (%d, %d, %d)\n", dimensionX, dimensionY, dimensionZ);

	printf("----------------------------------------------------------------------\n");
	printf("Source positions setting (0:default, 1:manual, 2:random) ");
	scanf("%d", &SourcePositionOption);
	printf("Sink positions setting (0:default, 1:manual, 2:random) ");
	scanf("%d", &SinkPositionOption);
	printf("----------------------------------------------------------------------\n");
}

void initializeParameters()
{
	packetSN = 0;
	depthThreshold = 0;
	numOfArrivals = 0;
	numOfDepartures = 0;
	nPositionUpdated = 0;
	totalCollisions = 0;
	currentTime = 0;
	bEndWithTime = true;
	SinkPositionOption = -1;
	SourcePositionOption = -1;
	executionTime = simulationTime;
	max_num_of_collision_in_one_slot = 0;
	collision_count = 0;

	for (int i = 0; i<numOfNodes; i++) {
		sensorNode[i].consumedEnergy = 0;
		sensorNode[i].currentEnergy = energy_MAX;
	}
}

void printToCheckNeighborNodes()
{
    for (int i = 0; i < numOfNodes; i++) {
        vector<NeighborNode> neighbors = scanNeighbors(i, transmissionRangeMax);
        fprintf(tFile1, "neighbors of node %d : \n", i);
        for (int j = 0; j < neighbors.size(); j++) {
            fprintf(tFile1, "node %d \n", neighbors[j].nodeID);
        }
    }
	fflush(tFile1);
}

void printToCheckChannelModelValue()
{
	for (float d = 100; d <= 1000; d += 100) {
		determineSendingSuccessByModels(d, true);
	}
	fflush(tFile0);
}

void generateInitialDepartureEventsForSource()
{
	// Generate the first packet departure event from the source nodes and push events into queue
	for (int i = 0; i<numOfSourceNodes; i++) {
		DBRpacket newPacket = DBRpacket(i, packetSN, sensorNode[i].nZ, 0);

		// source���ͪ��Ĥ@�ӫʥ]���ɶ��I����0��packet rate�����A�]CBR�ɶ����|�W�Lpacket rate
		sourceSendingTime[i] = get_rand(0.0, (float)sourcePacketRate);

		// �O��packet���ͪ��ɶ�
		packetRecordTable[packetSN].generatedTime = sourceSendingTime[i];

		// ��source node���ͲĤ@�ӫʥ]departure event
		EventItem newItem = EventItem(event_DEPARTURE, sourceSendingTime[i], i, newPacket);
		eventQueue.push_back(newItem);

		// �Ȯɤ���priority queue
		// updatePriorityQueue(i, newPacket.packetSN, sourceSendingTime[i]);
		updateHistoryBuffer(i, packetSN);

		packetSN++;
	}
}

bool generateNewDepartureEventsForSource(int iNode)
{
	// �Y�w��w�]���W��numOfPackets�A�N���A���͡A�@���u�{���פ�v���P�_����
	if (packetSN >= numOfPackets) return false;

	DBRpacket newPacket = DBRpacket(iNode, packetSN, sensorNode[iNode].nZ, 0);
	sourceSendingTime[iNode] += sourcePacketRate;
	packetRecordTable[packetSN].generatedTime = sourceSendingTime[iNode];   // �O�����ͪ��ɶ��A�̫�έp�ɨϥ�

	eventQueue.push_back(EventItem(event_DEPARTURE, sourceSendingTime[iNode], iNode, newPacket));

	// updatePriorityQueue(iNode, newPacket.packetSN, sourceSendingTime[iNode]); // �Ȯɤ���priority queue
	updateHistoryBuffer(iNode, packetSN);

	packetSN++;

	return true;
}

// 2016.11.28: �s�WupdateHistoryBuffer()
void updateHistoryBuffer(int nodeID, int packetSN)
{
	// if it is the first time that the node sends out the packet
	if (HistoryBuffer[nodeID][packetSN].bBuffered == false) {
		// update history buffer at this node (with this packet)
		HistoryBuffer[nodeID][packetSN].bBuffered = true;
	}
	/*
	// old version, unused
	if (!HistoryBuffer[eventQueue[eIndex].nodeID][eventQueue[eIndex].packet.packetSN].bBuffered)
	HistoryBuffer[eventQueue[eIndex].nodeID][eventQueue[eIndex].packet.packetSN].bBuffered = true;
	*/
}

int getNumberOfEventsInTimeslot(int currentTime)
{
	// sort items in the event queue
	sort(eventQueue.begin(), eventQueue.end(), EventSortOccurringTime);

	// check the number of events will be processed in this timeslot
	int iQueueSize = eventQueue.size();
	int numOfProcessingEvents = 0;

	// calculate how many events in this timeslot
	for (int i = 0; i<iQueueSize; i++) {
		if (eventQueue[i].occurringTime <= currentTime) {
			numOfProcessingEvents++;
			//eventQueue[i].occurringTime = currentTime;
		}
	}
	return numOfProcessingEvents;
}

int removeCollisionEvents(int nEvents)
{
	bool *bEventErase = (bool*)malloc(nEvents * sizeof(bool));
	for (int i = 0; i<nEvents; i++) {
		bEventErase[i] = false;
	}
	for (int i = 0; i<nEvents; i++) {
		for (int j = i + 1; j<nEvents; j++) {                      // j�_�l�Ȥ��൥��0�_�hi=0,j=0��if�N����  
			if (eventQueue[i].nodeID == eventQueue[j].nodeID &&
				eventQueue[i].eventType == event_ARRIVAL &&
				eventQueue[j].eventType == event_ARRIVAL &&
				eventQueue[i].occurringTime <= currentTime &&
				eventQueue[j].occurringTime <= currentTime) {  // ���󳣦��߮ɪ�ܵo��collision

				bEventErase[i] = true;
				bEventErase[j] = true;
			}
		}
	}
	int nCollisions = 0;                         // number of collisions
	for (int i = 0; i<nEvents; i++) {
		if (bEventErase[i]) {
			nCollisions++;
			eventQueue[i].occurringTime = -1;    // �]�o��collision��events���ɶ���-1
		}
	}
	sort(eventQueue.begin(), eventQueue.end(), EventSortOccurringTime);
	int iQueueSize = eventQueue.size();
	int tempIndex = -1;

	for (int i = 0; i<iQueueSize - 1; i++) {
		if (eventQueue[i].occurringTime == -1 &&
			eventQueue[i + 1].occurringTime != -1) {
			tempIndex = i + 1;                    // �n�R����events�ƶq
			break;
		}
	}

	// �N�o��collision��events�R��
	if (tempIndex != -1) {
		// �@���R���h��collision events
		eventQueue.erase(eventQueue.begin(), eventQueue.begin() + tempIndex);
	}

	free(bEventErase);

	return nCollisions;
}

void collisionPrinting(int nCollisions)
{
	if (nCollisions > 0)
		collision_count++;                              // �O�����h�֭�time slot�o��collision
	if (nCollisions > max_num_of_collision_in_one_slot) {
		max_num_of_collision_in_one_slot = nCollisions;
		fprintf(tFile4, "max_num_of_collision_in_one_slot: %d \n", max_num_of_collision_in_one_slot);
	}
	fflush(tFile4);
}

void dataPacketProcessing(int iEvent)
{
    if (eventQueue[iEvent].packet.packetSN ==1)   // �[�� packet #1 �����V (just for debugging)
        fprintf(tFile1, "packet %d (at depth %d) (from node %d) arrived at node %d when %d \n", 
		        eventQueue[iEvent].packet.packetSN, eventQueue[iEvent].packet.nodeDepth, 
				eventQueue[iEvent].packet.senderNodeID, eventQueue[iEvent].nodeID, 
				eventQueue[iEvent].occurringTime);

	// �T�{��node�Ѿl�q�O�O�_���������ӫʥ]�A�Y�����h���۶i��ʥ]���B�z
	float remainingEnergy = sensorNode[eventQueue[iEvent].nodeID].currentEnergy;
	if(remainingEnergy >= (powerReceiving - powerIdling)*timeslotLength) {

		// Energy consumption of receiving packet
		sensorNode[eventQueue[iEvent].nodeID].consumedEnergy += (powerReceiving - powerIdling)*timeslotLength;
		sensorNode[eventQueue[iEvent].nodeID].currentEnergy -= (powerReceiving - powerIdling)*timeslotLength;

		// �έpnumber of arrivals
		numOfArrivals++;

		// �����ʥ]��hop�ƼW�[�@��
		eventQueue[iEvent].packet.hop_count++;

		// �Y��event���O��Fsink��arrival event�A�h����DBR��forwarding algorithm
		if(!checkPacketArriveSink(iEvent)) {          
			DBR_PacketForwardingAlgorithm(iEvent);
		}
	}
	else {
		printf("Node[%d]'s energy is exhausted.\n", eventQueue[iEvent].nodeID);
		sensorNode[eventQueue[iEvent].nodeID].currentEnergy = -1;
	}
}

bool checkPacketArriveSink(int eIndex)
{
	if (sensorNode[eventQueue[eIndex].nodeID].nodeType == node_type_SINK) {  // �Y����Fsink��arrival event
		int SN = eventQueue[eIndex].packet.packetSN;
		if (!packetRecordTable[SN].isArrived) {                             // �Y�L�O����ܦ��s�����ʥ]�Ĥ@����sink�A�h�O����
			packetRecordTable[SN].isArrived = true;                         // (note:�ȰO���̦���Fsink�����@��)
			packetRecordTable[SN].arrivedTime = eventQueue[eIndex].occurringTime;
			packetRecordTable[SN].delay = packetRecordTable[SN].arrivedTime - packetRecordTable[SN].generatedTime;
			packetRecordTable[SN].hopCount = eventQueue[eIndex].packet.hop_count;  

			// note: �]�i�Hprint"generated time"�M"arrived time"���ˬd
			printf("Packet %d arrived node %d\n", eventQueue[eIndex].packet.packetSN, eventQueue[eIndex].nodeID);
			/*
			if (SN < numOfReferredPackets){
				printf("Packet %d arrived node %d \n", eventQueue[eIndex].packet.packetSN, eventQueue[eIndex].nodeID);//printf�Xdelay�ɶ�  by jerrywu
				printf("generate time %d arrived time %d delay time %d\n", packetRecordTable[SN].generatedTime, packetRecordTable[SN].arrivedTime, packetRecordTable[SN].delay);//printf�Xdelay�ɶ�  by jerrywu
			}
			*/
		}
		return true;
	}
	else return false;
}

void DBR_PacketForwardingAlgorithm(int eIndex)       // ���T�w�O�_��potential bug
{
	// �ˬdpacket���e�M�w��packet�n�Qforward��drop
	bool bDropPacket = checkReceivedPacketHeader(eventQueue[eIndex]);
	if (!bDropPacket) {  // �Y�q�L�ˬd(�Ddrop)

		DBRpacket newPacket = DBRpacket(eventQueue[eIndex].nodeID, eventQueue[eIndex].packet.packetSN,
			sensorNode[eventQueue[eIndex].nodeID].nZ, eventQueue[eIndex].packet.hop_count);

		// note: nodeID�Oreceiver��ID�A��nodeDepth�Osender��depth
		int delta_d = eventQueue[eIndex].packet.nodeDepth - sensorNode[eventQueue[eIndex].nodeID].nZ;
		int HT = computeHoldingTime(transmissionRangeMax, propogationSpeed, deltaValue, delta_d);
		int ST = currentTime + HT;

		// priority queue, unused
		// priorityQueueOperation(currentTime, ST, newPacket);

		// �ھ�sending time ST����departure event
		EventItem newItem = EventItem(event_DEPARTURE, ST, eventQueue[eIndex].nodeID, newPacket);
		eventQueue.push_back(newItem);

		// priority queue, unused
		// updatePriorityQueue(eventQueue[eIndex].nodeID, newPacket.packetSN, ST);

		// record the packet in history buffer
		updateHistoryBuffer(eventQueue[eIndex].nodeID, eventQueue[eIndex].packet.packetSN);

		//fprintf(tFile3, "current time, sending time ST, holding time HT: (%d, %d, %d) \n", currentTime, ST, HT);

		if (newPacket.packetSN == 10)
			fprintf(tFile5, "Departure Event: sending time, from which node, sequence number: (%d, %d, %d) \n",
			ST, eventQueue[eIndex].nodeID, newPacket.packetSN);
	}
}

bool checkReceivedPacketHeader(EventItem currentEvent) // ���T�w�O�_��potential bug
{
	// �ˬd�`�׮t(delta_d)�O�_�C��thresthold
	int previousDepth = sensorNode[currentEvent.packet.senderNodeID].nZ;
	int currentDepth = sensorNode[currentEvent.nodeID].nZ;

	// note: ������z�Ȭ�0�G�Y���U��ǡA�`�׮t�|�O�t����
	int delta_d = previousDepth - currentDepth;

	if (delta_d < depthThreshold) {
		return true;      // �Y�C��threshold�hdrop��
	}
	else {
		// �ˬdhistory buffer���O�_�����ʥ]�O��
		if (HistoryBuffer[currentEvent.nodeID][currentEvent.packet.packetSN].bBuffered) {
			return true;  // �Y���O���N����e�wforward�L��packet�hdrop��
		}
	}
	return false;
}

float computeHoldingTime(int transRangeMax, float propSpeed, float delta, int delta_d)
{
	float tau = transRangeMax / propSpeed;
	float holdingTime = (2 * tau / delta)*(transRangeMax - delta_d);
	return holdingTime;
}

void departurePacketProcessing(int iEvent)
{
	if (eventQueue[iEvent].packet.packetSN == 1)      // �[��packet #1�����V (just for debugging)
		fprintf(tFile1, "packet %d (from node %d) departured at node %d when %d \n",
		eventQueue[iEvent].packet.packetSN, eventQueue[iEvent].packet.senderNodeID,
		eventQueue[iEvent].nodeID, eventQueue[iEvent].occurringTime);

	// check the remaining energy of node
	float remainingEnergy = sensorNode[eventQueue[iEvent].nodeID].currentEnergy;
	if (remainingEnergy >= (powerSending - powerIdling)*timeslotLength) {

		// Energy consumption of forwarding packet
		sensorNode[eventQueue[iEvent].nodeID].consumedEnergy += (powerSending - powerIdling)*timeslotLength;   
		sensorNode[eventQueue[iEvent].nodeID].currentEnergy -= (powerSending - powerIdling)*timeslotLength;   

		// �έpnumber of departures
		numOfDepartures++;

		// departure event�o�͡A���;F��`�I��arrival events
		generateArrivalEventsForNeighbors(iEvent);
	}
	else {
		printf("Node[%d]'s energy is exhausted.\n", eventQueue[iEvent].nodeID);
		sensorNode[eventQueue[iEvent].nodeID].currentEnergy = -1;
	}
}

void generateArrivalEventsForNeighbors(int eIndex)
{
	// �b�ǿ�b�|�����`�I���F��`�I
	vector<NeighborNode> neighbors = scanNeighbors(eventQueue[eIndex].nodeID, transmissionRangeMax);
	int iNeighborSize = neighbors.size();
	for (int n = 0; n<iNeighborSize; n++) {
		// check sending success by the probability from the channel model
		bool sendingSuccess = determineSendingSuccessByModels(neighbors[n].nodeDistance, false);
		if (sendingSuccess) {
			// generate arrival events of successful sending for neighbor nodes
			int delay = neighbors[n].propogationDelay + neighbors[n].transmissionDelay;
			int AT = currentTime + delay;

			// �ھ�arrival time AT���;F��`�I��arrival event
			// ��: �ʥ]�̪�senderID�MnodeDepth���O�ϥθ�departure event�o�ͪ�node
			DBRpacket newPacket = DBRpacket(eventQueue[eIndex].nodeID, eventQueue[eIndex].packet.packetSN,
				sensorNode[eventQueue[eIndex].nodeID].nZ, eventQueue[eIndex].packet.hop_count);   
			eventQueue.push_back(EventItem(event_ARRIVAL, AT, neighbors[n].nodeID, newPacket));

			//fprintf(tFile3, "current time, delay, arrival time AT: (%d, %d, %d) \n", currentTime, delay, AT);

			if (newPacket.packetSN == 10)
				fprintf(tFile6, "Arrival Event: arrival time, from which node, to which node, sequence number: (%d, %d, %d, %d) \n",
				AT, eventQueue[eIndex].nodeID, neighbors[n].nodeID, newPacket.packetSN);
		}
	}
}

vector<NeighborNode> scanNeighbors(int nodeID, int txRange)
{
    // index: node serial number
    vector<NeighborNode> neighborTable;
    DBRnode tempNode = sensorNode[nodeID];

    for (int n = 0; n<numOfNodes; n++) {
        if(n == nodeID) continue;    // �Y�������h���L�ð���U�@���j��
        int d2_X = pow(abs(tempNode.nX - sensorNode[n].nX), 2.0);
        int d2_Y = pow(abs(tempNode.nY - sensorNode[n].nY), 2.0);
        int d2_Z = pow(abs(tempNode.nZ - sensorNode[n].nZ), 2.0);
        int d2 = d2_X + d2_Y + d2_Z; // x^2 + y^2 + z^2
        int R2 = pow(txRange, 2.0);
        float d = pow(d2, 0.5);      // �}�ڸ�

        // if the node is within the transmission range, push it as item in table 
        if (d2 <= R2) {
            neighborTable.push_back(NeighborNode(n, sensorNode[n].nZ, d));
        }
    }
    return neighborTable;
}

// ------------------------------------------------------------------
void removeEvents(int numberOfProcessedEvents)
{
	// erase the events that already processed
	if (numberOfProcessedEvents == 0) return;
	else eventQueue.erase(eventQueue.begin(), eventQueue.begin() + numberOfProcessedEvents);
}
// ------------------------------------------------------------------

void printNodeEnergyCurrentTimeslot(int timeslot)
{
	fprintf(tFile9, "timeslot = %d\n", timeslot);
	for (int i = 0; i<numOfNodes; i++) {
		fprintf(tFile9, "Node %d consumed energy = %e, current energy = %e\n", i, sensorNode[i].consumedEnergy, sensorNode[i].currentEnergy);
	}
	fflush(tFile9);
}

void printCenterTableCurrentTimeslot(int timeslot)
{
	fprintf(tFile11, "timeslot = %d\n", timeslot);
	for (int i = 0; i<numOfNodes; i++) {
		fprintf(tFile11, "Node %d position = (%d,%d,%d)\n", i,
			nodeRecordTable[i].position_X, nodeRecordTable[i].position_Y, nodeRecordTable[i].position_Z);
	}
	fflush(tFile11);
}

void idleEnergyConsumption()
{
	// Energy consumption of node idling
	for (int i = 0; i<numOfNodes; i++) {
		float remainingEnergy = sensorNode[i].currentEnergy;
		if (remainingEnergy >(powerIdling*timeslotLength)) {
			sensorNode[i].consumedEnergy += (powerIdling*timeslotLength);   
			sensorNode[i].currentEnergy -= (powerIdling*timeslotLength);    
		}
		else {
			printf("Node[%d]'s energy exhausted.\n", i);
			sensorNode[i].currentEnergy = -1;
		}
	}
}

void checkAndGenerateNewPacket()
{
	if (!bEndWithTime) return;

	// check every source node which should generate a new data packet
	for (int i = 0; i<numOfSourceNodes; i++) {
		if (sourceSendingTime[i] + sourcePacketRate <= currentTime) {    
			if (!generateNewDepartureEventsForSource(i)) {
				// �qsource node���ͷs��departure event�A�u���b�u�ʥ]�ƶq��F�W���v�ɤ~�^��false�A�_�h�^��true
				// �Y�u���b�̫�@�ӫʥ]�e�X�ɡA�~�|�i�J
				bEndWithTime = false;
				executionTime = currentTime + extendedTime;
			}
		}
	}
}

void checkNodePositionUpdate()
{
	// update node positions periodically
	if (currentTime == (nPositionUpdated * positionUpdateSlot)) {
		updateRelayNodePosition();
		nPositionUpdated++;                            // ��n��update nodes' position
	}
}

void updateRelayNodePosition()   // �U��V���ӳt�v random ����
{
	// ��: ���]sink�Msource nodes�����|���� 
    for(int n = numOfSinkNodes + numOfSourceNodes; n<numOfNodes; n++) {

        // calculate node move distance based on the move speed
        float speed_x = get_rand(nodeSpeedMin, nodeSpeedMax);
        float speed_y = get_rand(nodeSpeedMin, nodeSpeedMax);
        float speed_z = get_rand(nodeSpeedMin, nodeSpeedMax);
        float detX = speed_x * positionUpdateSlot;
        float detY = speed_y * positionUpdateSlot;
        float detZ = speed_z * positionUpdateSlot;

        // update the node position
        bool bPositiveDirection = rand() % 2;  // ���M�w��V�O"��"��"�t"
        if(bPositiveDirection) {
            if(sensorNode[n].nX + detX <= dimensionX) sensorNode[n].nX += detX;
            else sensorNode[n].nX = dimensionX;
        }
        else {
            if(sensorNode[n].nX - detX > 0) sensorNode[n].nX -= detX;
            else sensorNode[n].nX = 0;
        }

        bPositiveDirection = rand() % 2;
        if(bPositiveDirection) {
            if(sensorNode[n].nY + detY <= dimensionY) sensorNode[n].nY += detY;
            else sensorNode[n].nY = dimensionY;
        }
        else {
            if(sensorNode[n].nY - detY > 0) sensorNode[n].nY -= detY;
            else sensorNode[n].nY = 0;
        }

        bPositiveDirection = rand() % 2;
        if(bPositiveDirection) {
            if(sensorNode[n].nZ + detZ <= dimensionZ) sensorNode[n].nZ += detZ;
            else sensorNode[n].nZ = dimensionZ;
        }
        else {
            if(sensorNode[n].nZ - detZ > 0) sensorNode[n].nZ -= detZ;
            else sensorNode[n].nZ = 0;
        }
		// update the central record table
		nodeRecordTable[n].position_X = sensorNode[n].nX;
		nodeRecordTable[n].position_Y = sensorNode[n].nY;
		nodeRecordTable[n].position_Z = sensorNode[n].nZ;
    }
}
// ------------------------------------------------------------------

void calculateStatistics()
{
    printf("Number of arrived packets = %f\n", calculateNumOfArrived());

    printf("Delivery ratio = %f\n", calculateDeliveryRatio());
    printf("Energy consumption = %f (J)\n", calculateEnergyConsumption());
    printf("Total delay of arrived packets = %f (ms)\n", calculateTotalDelay());
    printf("Total Collisions = %d (times)\n", totalCollisions);
    
    printf("Delay of non-arrived packets = %f\n", calculateDelayOfNonArrived());
    printf("Total delay of both kinds of packets = %f\n", calculateTotalDelay() + calculateDelayOfNonArrived());
    printf("Average delay of both kinds of packets = %f\n", (calculateTotalDelay() + calculateDelayOfNonArrived()) / calculateNumOfArrived());

    printPacketHopCount();   // hop count of each packet

    fprintf(tFile4, "Total Collisions = %d (times)\n", totalCollisions);
    fprintf(tFile4, "number of time slots that Collisions happen = %d (times)\n", collision_count);
	fflush(tFile4);
}

float calculateNumOfArrived()
{
    int NumofArrived = 0;
    for (int n = 0; n<numOfReferredPackets; n++) {
        if (packetRecordTable[n].isArrived) NumofArrived++;
    }
    return (float)NumofArrived;
}

float calculateDeliveryRatio()
{
    int numOfDelivery = 0;
    for(int n=0; n<numOfReferredPackets; n++) {
        if(packetRecordTable[n].isArrived) numOfDelivery++;
    }
    return (float)numOfDelivery/numOfReferredPackets;
}

float calculateEnergyConsumption()
{
    float idleConsumptionRelayNodes = numOfRelayNodes*powerIdling*executionTime; 
	printf("idleConsumptionRelayNodes = %f (J)\n", idleConsumptionRelayNodes);
    
	float idleConsumptionAllNodes = numOfNodes*powerIdling*executionTime; 
	printf("idleConsumptionAllNodes = %f (J)\n", idleConsumptionAllNodes);

    float sendingConsumption = numOfDepartures*(powerSending - powerIdling)*timeslotLength;
    float receivingConsumption = numOfArrivals*(powerReceiving - powerIdling)*timeslotLength;

    return (idleConsumptionRelayNodes + sendingConsumption + receivingConsumption);
}

float calculateTotalDelay()
{
    float totalDelay = 0;
    for(int d=0; d<numOfReferredPackets; d++) {
        if(packetRecordTable[d].delay != INT_MAX) {
            totalDelay += packetRecordTable[d].delay;
        }
    }  
    return totalDelay;
}

float calculateDelayOfNonArrived()
{
    int totalDelayofNonArrived = 0;

    for (int n = 0; n<numOfReferredPackets; n++) {
        if (!packetRecordTable[n].isArrived) {
            // method 1 (not for short simulation time)
            totalDelayofNonArrived += (currentTime - packetRecordTable[n].generatedTime); 
        }
    }
    return (float)totalDelayofNonArrived;
}

void printPacketHopCount()
{
	for (int i = 0; i<numOfPackets; i++) {
		fprintf(tFile8, "Packet %d hop count = %d\n", i, packetRecordTable[i].hopCount);
	}
	fflush(tFile8);
}

void endingMessage()
{
	printf("executionTime = %d (ms)\n", executionTime);
	printf("currentTime = %d (ms)\n", currentTime);

	if (bEndWithTime) printf("Simulation ended with time.\n");
	else printf("Simulation ended with the last packet.\n");
}

// Print information in the log files~!!
void closeLogFiles()
{
	fclose(tFile1);
	fclose(tFile2);
	fclose(tFile3);
	fclose(tFile4);
	fclose(tFile5);
	fclose(tFile6);
	fclose(tFile7);
	fclose(tFile8);
	fclose(tFile9);
	fclose(tFile10);
	fclose(tFile11);
}

// 2016.11.28: �s�WupdatePriorityQueue()
void updatePriorityQueue(int nodeID, int packetSN, int occuringTime) 
{
    // �Ȯɥ�����@
}

void priorityQueueOperation(int currentTime, int &ST, DBRpacket newPacket)
{
	// 2016.11.23 find a bug: ST_p may be less than currentTime (���i�H) 
	// �Ȯɤ���@ priority queue (Q1) �Pı�� bug    
}

// ------------------------------------------------------------------
// �H�����ͤ���lower�Mupper��������
// ------------------------------------------------------------------
float get_rand(float lower, float upper)
{
    return rand() * (upper-lower) / RAND_MAX + lower;
}

// ------------------------------------------------------------------
// ��Xsensor nodes���y�Ц�m(�ù�/�ɮ�)
// ------------------------------------------------------------------
void trace(bool bIsFile)
{
    if (bIsFile) {
        FILE *tFile = fopen(traceFileName_1, "w");
        fprintf(tFile, "packet length: %d\n", packetDataLength + sizeof(int)* 3);
        for (int n = 0; n<numOfNodes; n++) {
            fprintf(tFile, "sensor node[%d] = %d, %d, %d\n", n, sensorNode[n].nX, sensorNode[n].nY, sensorNode[n].nZ);
        }
        fclose(tFile);
    }
    else {
        printf("packet length: %d\n", packetDataLength + sizeof(int)* 3);
        for (int n = 0; n<numOfNodes; n++) {
            printf("sensor node[%d] = %d, %d, %d\n", n, sensorNode[n].nX, sensorNode[n].nY, sensorNode[n].nZ);
        }
    }
}

//--------------------------------------------
// �]�� position function ����, �ҥH��b���᭱
//--------------------------------------------
void initializeNodePosition()
{
    int n = 0;
    for (; n<numOfSourceNodes; n++) {                   // for source nodes
        if (SourcePositionOption == 0) {                // case 0: default setting
            if (numOfSourceNodes == 1){
                sensorNode[0].nX = (dimensionX / 2);
                sensorNode[0].nY = (dimensionY / 2);
            }
            /*else if (numOfSourceNodes == 2){
                sensorNode[0].nX = (dimensionX / 2);
                sensorNode[0].nY = (dimensionY / 2);
                sensorNode[1].nX = (dimensionX / 4);
                sensorNode[1].nY = (dimensionY / 4);
            }*/
            // �H���i������ numOfSourceNodes == 9 (�ثe����1��source node�Y�i)
        }
        else if (SourcePositionOption == 1) {           // case 1: manual input
            int sourceID = n ;
            printf("Enter the position X of source node %d = ", sourceID);
            scanf("%d", &sensorNode[n].nX);
            printf("Enter the position Y of source node %d = ", sourceID);
            scanf("%d", &sensorNode[n].nY);
        }
        else if (SourcePositionOption == 2){            // case 2: random
            sensorNode[n].nX = rand() % (dimensionX + 1);
            sensorNode[n].nY = rand() % (dimensionY + 1);
        }
        sensorNode[n].nZ = dimensionZ;                  // bottom of the sea
        sensorNode[n].nodeType = node_type_SOURCE;

		fprintf(tFile10, "source node id_%d = (%d,%d,%d)\n", n, sensorNode[n].nX, sensorNode[n].nY, sensorNode[n].nZ);
    }

    for(; n<numOfSourceNodes+numOfSinkNodes; n++) {     // for sink nodes
        if (SinkPositionOption == 0) {                  // case 0: default setting
            setSinkDefaultPositions();
        }
        else if(SinkPositionOption == 1) {              // case 1: manual input
            int sinkID = n - numOfSourceNodes;             
            printf("Enter the position X of sink node %d = ", sinkID);
            scanf("%d",&sensorNode[n].nX);
            printf("Enter the position Y of sink node %d = ", sinkID);
            scanf("%d",&sensorNode[n].nY);
        }
        else if(SinkPositionOption == 2){               // case 2: random
            sensorNode[n].nX = rand() % (dimensionX+1);
            sensorNode[n].nY = rand() % (dimensionY+1);
        }
        sensorNode[n].nZ = 0;                           // surface of the sea
        sensorNode[n].nodeType = node_type_SINK;

		fprintf(tFile10, "sink node id_%d = (%d,%d,%d)\n", n, sensorNode[n].nX, sensorNode[n].nY, sensorNode[n].nZ);
    }

    for (; n < numOfSourceNodes + numOfSinkNodes + numOfRelayNodes; n++) {    // for remaining relay nodes
        sensorNode[n].nX = rand() % (dimensionX + 1);         
        sensorNode[n].nY = rand() % (dimensionY + 1);
        sensorNode[n].nZ = rand() % (dimensionZ + 1);
        sensorNode[n].nodeType = node_type_RELAY;

		fprintf(tFile10, "relay node id_%d = (%d,%d,%d)\n", n, sensorNode[n].nX, sensorNode[n].nY, sensorNode[n].nZ);
    }

    nPositionUpdated++;

	// update the central record table
	for(int i=0; i<numOfNodes; i++) {
		nodeRecordTable[i].position_X = sensorNode[i].nX;
		nodeRecordTable[i].position_Y = sensorNode[i].nY;
		nodeRecordTable[i].position_Z = sensorNode[i].nZ;
	}
}

void setSinkDefaultPositions()     // by chu 
{                                  // by jerry wu
	if (numOfSinkNodes == 1){
		sensorNode[numOfSourceNodes].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes].nY = (dimensionY / 2);
	}
	else if (numOfSinkNodes == 2){
		sensorNode[numOfSourceNodes].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 1].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 1].nY = (dimensionY / 2);
	}
	else if (numOfSinkNodes == 3){
		sensorNode[numOfSourceNodes].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 1].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 1].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 2].nX = (dimensionX / 4);
		sensorNode[numOfSourceNodes + 2].nY = (dimensionY / 2);
	}
	else if (numOfSinkNodes == 4){
		sensorNode[numOfSourceNodes].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 1].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 1].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 2].nX = (dimensionX / 4);
		sensorNode[numOfSourceNodes + 2].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 3].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes + 3].nY = (dimensionY / 4);
	}
	else if (numOfSinkNodes == 5){
		sensorNode[numOfSourceNodes].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 1].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 1].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 2].nX = (dimensionX / 4);
		sensorNode[numOfSourceNodes + 2].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 3].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes + 3].nY = (dimensionY / 4);
		sensorNode[numOfSourceNodes + 4].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes + 4].nY = 3 * (dimensionY / 4);
	}
	else if (numOfSinkNodes == 6){
		sensorNode[numOfSourceNodes].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 1].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 1].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 2].nX = (dimensionX / 4);
		sensorNode[numOfSourceNodes + 2].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 3].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes + 3].nY = (dimensionY / 4);
		sensorNode[numOfSourceNodes + 4].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes + 4].nY = 3 * (dimensionY / 4);
		sensorNode[numOfSourceNodes + 5].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 5].nY = 3 * (dimensionY / 4);
	}
	else if (numOfSinkNodes == 7){
		sensorNode[numOfSourceNodes].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 1].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 1].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 2].nX = (dimensionX / 4);
		sensorNode[numOfSourceNodes + 2].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 3].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes + 3].nY = (dimensionY / 4);
		sensorNode[numOfSourceNodes + 4].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes + 4].nY = 3 * (dimensionY / 4);
		sensorNode[numOfSourceNodes + 5].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 5].nY = 3 * (dimensionY / 4);
		sensorNode[numOfSourceNodes + 6].nX = (dimensionX / 4);
		sensorNode[numOfSourceNodes + 6].nY = (dimensionY / 4);
	}
	else if (numOfSinkNodes == 8){
		sensorNode[numOfSourceNodes].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 1].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 1].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 2].nX = (dimensionX / 4);
		sensorNode[numOfSourceNodes + 2].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 3].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes + 3].nY = (dimensionY / 4);
		sensorNode[numOfSourceNodes + 4].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes + 4].nY = 3 * (dimensionY / 4);
		sensorNode[numOfSourceNodes + 5].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 5].nY = 3 * (dimensionY / 4);
		sensorNode[numOfSourceNodes + 6].nX = (dimensionX / 4);
		sensorNode[numOfSourceNodes + 6].nY = (dimensionY / 4);
		sensorNode[numOfSourceNodes + 7].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 7].nY = (dimensionY / 4);
	}
	else if (numOfSinkNodes == 9){
		sensorNode[numOfSourceNodes].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 1].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 1].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 2].nX = (dimensionX / 4);
		sensorNode[numOfSourceNodes + 2].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 3].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes + 3].nY = (dimensionY / 4);
		sensorNode[numOfSourceNodes + 4].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes + 4].nY = 3 * (dimensionY / 4);
		sensorNode[numOfSourceNodes + 5].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 5].nY = 3 * (dimensionY / 4);
		sensorNode[numOfSourceNodes + 6].nX = (dimensionX / 4);
		sensorNode[numOfSourceNodes + 6].nY = (dimensionY / 4);
		sensorNode[numOfSourceNodes + 7].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 7].nY = (dimensionY / 4);
		sensorNode[numOfSourceNodes + 8].nX = (dimensionX / 4);
		sensorNode[numOfSourceNodes + 8].nY = 3 * (dimensionY / 4);
	}
}

//----------------------------------------------------
// channel model ���W�ߪ��@�����A�B�D�`���n, ��b�̫᭱~!!
//----------------------------------------------------
bool determineSendingSuccessByModels(float nodeDistance, bool printValue)
{
	// powerSending (unit: J/ms) ---> transmitter_power (unit: J/s)
	float transmitter_power = powerSending * 1000;

	// frequency = 10kHz
	float f = 10;

	// a(f): absorption coefficient in dB/km
	float a_f = 0.11*pow(f, 2) / (1 + pow(f, 2)) + 44 * pow(f, 2) / (4100 + pow(f, 2)) + 2.75*pow(10.0, -4)*pow(f, 2) + 0.003; // �l���Y��

	// node distance, radius of an imaginary sphere
	float d = nodeDistance;
	float r = d;

	// source level, transmission loss, noise level, directivity index
	float T_loss = 20 * log10(d) + a_f*d*pow(10.0, -3);
	float S_level = 10 * (log10(transmitter_power) - log10(4 * M_PI*pow(r, 2)) - log10(0.67*pow(10.0, -18)));
	float N_level = 50 - 18 * log10(f);
	float D_index = 0;

	// Signal to Noise Ratio (SNR)
	float gamma = S_level - T_loss - N_level + D_index;

	// Bit Error Rate (BER)
	float BER = 0.5*(1 - sqrt(pow(10, (float)gamma / 10) / (1 + pow(10, (float)gamma / 10))));
	if (printValue) {
		fprintf(tFile0, "transmitter_power = %f, nodeDistance = %f, BER = %e\n", transmitter_power, nodeDistance, BER);
	}

	// successProbability ~= 50% (1W, 300m, 49.5% / 2W, 350m, 51.6%)
	float successProbability = pow((float)1 - BER, (float)packetSize * 8);

	// �H����0~1��������
	float _rand = get_rand(0.0, 1.0);

	if (_rand <= successProbability)
		return true;             // �ǿ馨�\
	else
		return false;            // �ǿ饢��
}