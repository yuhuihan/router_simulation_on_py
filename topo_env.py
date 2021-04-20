# -*- coding: utf-8 -*-
"""
Created on Tue Mar 23 17:10:53 2021

@author: yhh18
"""
import numpy as np
import json


class TOPOENV:

    def __init__(self, filename='missions.json'):

        # # 邻接矩阵
        # topo = [0, 1, 1, 1,
        #         -1, 0, -1, 1,
        #         -1, -1, 0, 1,
        #         -1, -1, -1, 0]
        # # 概率矩阵
        # probability = [0, 40, 40, 20,
        #                0, 0, 0, 100,
        #                0, 0, 0, 100,
        #                0, 0, 0, 0]
        # # 路由任务
        ##[{"src": 0, "dst": 3, "rate": 3}]

        # topo = [0,0,1,0,0,0,
        #         0,0,1,0,0,0,
        #         0,0,0,1,0,0,
        #         0,0,0,0,1,1,
        #         0,0,0,0,0,0,
        #         0,0,0,0,0,0]
        # probability = [0,0,1,0,0,0,
        #                 0,0,1,0,0,0,
        #                 0,0,0,1,0,0,
        #                 0,0,0,0,1,1,
        #                 0,0,0,0,0,0,
        #                 0,0,0,0,0,0]

        # [{"src": 0, "dst": 5, "rate": 2},
        # {"src": 1, "dst": 4, "rate": 1}]

        # topo = [0, 1, 1, 0, 0, 0, 0,
        #         0, 0, 0, 1, 0, 0, 0,
        #         0, 0, 0, 1, 0, 0, 0,
        #         0, 0, 0, 0, 1, 0, 0,
        #         0, 0, 0, 0, 0, 1, 1,
        #         0, 0, 0, 0, 0, 0, 0,
        #         0, 0, 0, 0, 0, 0, 0]
        # probability = [0, 4, 2, 0, 0, 0, 0,
        #                 0, 0, 0, 1, 0, 0, 0,
        #                 0, 0, 0, 1, 0, 0, 0,
        #                 0, 0, 0, 0, 1, 0, 0,
        #                 0, 0, 0, 0, 0, 1, 1,
        #                 0, 0, 0, 0, 0, 0, 0,
        #                 0, 0, 0, 0, 0, 0, 0]
        ##[{"src": 0, "dst": 5, "rate": 3}]

        self.packetLength = 512 * 8  # 包长
        self.miu = 5*1024*1024 / self.packetLength  # 服务率
        self.lamada = []  # 到达率
        self.srcNode = []  # 路由开始节点
        self.dstNode = []  # 路由结束节点
        topo = []
        filename = 'missions.json'
        with open(filename) as f:
            mission_list = json.load(f)
            topo = mission_list['topo'].copy()
        for mission in mission_list['mission']:
            self.lamada.append(mission['rate']*1024*1024 / self.packetLength)
            self.srcNode.append(mission['src'])
            self.dstNode.append(mission['dst'])
        self.nodeNum = int(np.sqrt(len(topo)))
        self.topo = np.zeros((self.nodeNum, self.nodeNum), int)
        self.probability = np.zeros((self.nodeNum, self.nodeNum), int)
        self.reach = np.zeros((self.nodeNum, self.nodeNum), int)
        for i in range(0, self.nodeNum):
            for j in range(0, self.nodeNum):
                self.topo[i][j] = topo[i*self.nodeNum + j]
        self.SumLinkLamada = np.zeros((self.nodeNum, self.nodeNum), float)

    def step(self, action):
        for i in range(0, self.nodeNum):
            for j in range(0, self.nodeNum):
                self.probability[i][j] = action[i*self.nodeNum + j]
                self.reach[i][j] = self.topo[i][j] * self.probability[i][j]
        self.getReachableMx()
        reward = self.getDelay()
        reward = -reward * 1000
        return reward
        
    def getReachableMx(self):
        self.dist = np.zeros((self.nodeNum, self.nodeNum), int)
        for i in range(0, self.nodeNum):
            for j in range(0, self.nodeNum):
                if self.reach[i][j] != 0 or i == j:
                    self.dist[i][j] = 1
        for k in range(0, self.nodeNum):
            for i in range(0, self.nodeNum):
                for j in range(0, self.nodeNum):
                    self.dist[i][j] = self.dist[i][j] or (
                        self.dist[i][k] and self.dist[k][j])
        return self.dist

    def getNextNode(self, nodeId, dstNode, lastNode=-1):
        # lastNode 防止两节点成环
        probSum = 0
        candidateNodes = []
        original_candidateProbs = []
        new_candidateProbs = []
        for i in range(0, self.nodeNum):
            if self.reach[nodeId][i] and self.dist[i][dstNode] and lastNode != i:
                candidateNodes.append(i)
                original_candidateProbs.append(self.probability[nodeId][i])
                probSum = probSum + self.probability[nodeId][i]
        for i in range(len(candidateNodes)):
            new_probability = original_candidateProbs[i] / probSum
            new_candidateProbs.append(new_probability)
        return candidateNodes, new_candidateProbs

    def getLinkLamada(self, srcNode, dstNode, lamada):
        linkProb = np.zeros((self.nodeNum), float)
        nodeProb = np.zeros((self.nodeNum), float)
        linkProbs = np.zeros((self.nodeNum, self.nodeNum), float)
        lastNode = - np.ones((self.nodeNum), float)
        linkLamada = np.zeros((self.nodeNum, self.nodeNum), float)
        linkProb[srcNode] = 1
        Nodes = []
        first_candidateNodes = []
        first_new_candidateProbs = []
        count = 0
        if srcNode != dstNode:
            Nodes.append(srcNode)
        while Nodes:
            nodeId = Nodes[-1]
            candidateNodes, new_candidateProbs = self.getNextNode(
                nodeId, dstNode, lastNode[nodeId])
            Nodes.pop()
            i = 0
            # 防止链路下一跳进入一个由两节点环路构成的支路
            while i < len(candidateNodes):
                if candidateNodes[i] != dstNode:
                    next_candidateNodes, next_new_candidateProbs = self.getNextNode(
                        candidateNodes[i], dstNode, nodeId)
                    if len(next_candidateNodes) == 0:
                        candidateNodes.pop(i)
                        new_candidateProbs.pop(i)
                        i = i - 1
                i = i + 1
            for i in range(len(candidateNodes)):
                lastNode[candidateNodes[i]] = nodeId
            SumProb = 0
            Probs = []
            for k in new_candidateProbs:
                SumProb = SumProb + k
            for k in new_candidateProbs:
                z = k / SumProb
                Probs.append(z)  # 概率重新归一化
            # 记录始发节点的任务
            if count == 0:
                for i in candidateNodes:
                    first_candidateNodes.append(i)
                for i in Probs:
                    first_new_candidateProbs.append(i)
                sum_dst_candidateProbs = first_new_candidateProbs[-1]
            # 计算节点的流量
            for i in range(len(candidateNodes)):
                linkProb[candidateNodes[i]] = linkProb[nodeId] * Probs[i]
                nodeProb[candidateNodes[i]] = nodeProb[candidateNodes[i]
                                                       ] + linkProb[candidateNodes[i]]
                linkProbs[nodeId][candidateNodes[i]
                                  ] = linkProbs[nodeId][candidateNodes[i]] + linkProb[candidateNodes[i]]
                if candidateNodes[i] != dstNode:
                    Nodes.append(candidateNodes[i])
            count = count + 1
            # 本算法依次计算始发节点的任务
            if nodeProb[dstNode] > sum_dst_candidateProbs * 0.99:
                first_candidateNodes.pop()
                first_new_candidateProbs.pop()
                if not first_candidateNodes:
                    break
                sum_dst_candidateProbs = sum_dst_candidateProbs + \
                    first_new_candidateProbs[-1]

                # Nodes = []
                # for m in first_candidateNodes:
                #     Nodes.append(m)
                # lastNode[Nodes[-1]] = srcNode
                # linkLamada = np.zeros((self.nodeNum),float)
                # linkLamada[srcNode] = 1
                # linkLamada[first_candidateNodes[-1]] = first_new_candidateProbs[-1] * linkLamada[srcNode]

            # if count == 2000:
            #     break
        for i in range(0, self.nodeNum):
            for j in range(0, self.nodeNum):
                linkLamada[i][j] = linkProbs[i][j] * lamada
        return linkLamada

    def getDelay(self):
        for i in range(len(self.srcNode)):
            linkLamada = self.getLinkLamada(
                self.srcNode[i], self.dstNode[i], self.lamada[i])
            for j in range(0, self.nodeNum):
                for k in range(0, self.nodeNum):
                    self.SumLinkLamada[j][k] = self.SumLinkLamada[j][k] + \
                        linkLamada[j][k]
        T_sum = 0
        SumLamada = 0
        for i in range(0, self.nodeNum):
            for j in range(0, self.nodeNum):
                if self.SumLinkLamada[i][j] != 0:
                    T_sum = T_sum + \
                        (1 / (self.miu - self.SumLinkLamada[i][j])
                         + 0.002) * self.SumLinkLamada[i][j]
                SumLamada = sum(self.lamada)
        T_average = T_sum / SumLamada
        return T_average
