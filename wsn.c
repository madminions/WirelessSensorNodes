#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>

int N = 10000;
int mod = 500;

char DAT_FILE[100]; /* loactions of sensor nodes in the target field to be stored in a data file */
int KEYRING_SIZE; /* key ring size m for each sensor node */
int KEYPOOL_SIZE; /* key pool size M */

int n; /* total number of sensor nodes to be deployed */
int d; /* average number of neighbor nodes for each sensor node */
double RANGE; /* communication range of sensor node */
double p; /* network connectivity during direct key establishment phase */
double pi; /* network connectivity during path key establishment phase using i hops in the path */

/* a sensor node representation */
typedef struct {
	int x; /* x-coordinate of sensor node location in target field */
	int y; /* y-coordinate of sensor node location in target field */
	int *keyring; /* key ring */
	int phynbrsize; /* number of physical neighbors of a sensor node */
	int keynbrsize; /* number of key neighbors of a sensor node */
	int *phynbr; /* List of physical neighbors of a sensor node */
	int *keynbr; /* List of key neighbors of a sensor node */
} sensor;

int isPresent(int array[],int size,int item);

int main(int argc,char *argv[])
{
	if(argc!=4)
	{
		printf("Not sufficient arguments \n");
		exit(0);
	}
	printf("Reading Data FILE....\nScaling communication range\n");
	time_t t;
	int i,j,tempx,tempy;
	KEYRING_SIZE=atoi(argv[3]);
	KEYPOOL_SIZE=atoi(argv[2]);

	double dist,opdist=0;
	sensor nodes[10000];
	int check[500][500]={0};
	int hop2node[10000]={0};
	int count=10000;

	// initialization
	for(i=0;i<N;i++)
	{
		nodes[i].keyring = malloc(100*sizeof(int));
		nodes[i].phynbrsize = 0;
		nodes[i].keynbrsize = 0;
		nodes[i].phynbr = malloc(100*sizeof(int));
		nodes[i].keynbr = malloc(100*sizeof(int));
	}
	
	strcpy(DAT_FILE,argv[1]);
	FILE *fpDatFIle = fopen(DAT_FILE,"w");
	FILE *fpPlot = fopen("sensors.gp","w");

	srand((unsigned) time(&t));
	i=0;
	// generating unique random points
	while(count>0)
	{
		tempx = rand()%mod;
		tempy = rand()%mod;
		if(check[tempx][tempy] == 0)
			nodes[i].x=tempx,nodes[i++].y=tempy,count--,check[tempx][tempy]=1;
	}

	//writing to DAT file
	for(i=0;i<N;i++)
		fprintf(fpDatFIle, "%d %d\n", nodes[i].x,nodes[i].y);

	fprintf(fpPlot, "plot '%s'", DAT_FILE);
	fclose(fpDatFIle);
	fclose(fpPlot);

	system("gnuplot -p 'sensors.gp'");

	// average distance
	for(i=0;i<N;i++)
	{
		dist=0.0;
		for(j=0;j<N;j++)
			dist += sqrt( (nodes[i].x - nodes[j].x)*(nodes[i].x - nodes[j].x) + 
				(nodes[i].y - nodes[j].y)*(nodes[i].y - nodes[j].y) ) ;
		opdist = opdist + dist/(N-1);
	}
	double avgdist = opdist/N;

	RANGE = avgdist/10;
	
	printf("Average Distance = %lf\n", avgdist);
	printf("\nCommunication range of sensor nodes = %lf\n", RANGE);

	// distributing keys
	for(i=0;i<N;i++)
	{
		for(j=0;j<KEYRING_SIZE;j++)
			nodes[i].keyring[j] = rand()%KEYPOOL_SIZE;
	}
	int u_i,v_i,flag;
	printf("\nComputing physical neighbours.... \n");
	
	// direct key establishment phase
	for(i=0;i<N;i++)
	{
		for(j=0;j<N;j++)
		{
			// checking physical neighbour
			if(sqrt( (nodes[i].x - nodes[j].x)*(nodes[i].x - nodes[j].x) + 
				(nodes[i].y - nodes[j].y)*(nodes[i].y - nodes[j].y) ) <= RANGE && j!=i)
			{
				nodes[i].phynbr[nodes[i].phynbrsize++] = j;

				flag=0;
				//checking for key neighbour
				for(u_i=0;u_i<KEYRING_SIZE;u_i++)
				{
					for(v_i=0;v_i<KEYRING_SIZE;v_i++)
						if(nodes[i].keyring[u_i]==nodes[j].keyring[v_i])
							{
								flag=1;
								nodes[i].keynbr[nodes[i].keynbrsize++]=j;
								break;
							}
					if(flag)
						break;
				}
			}
		}
	}

	long int num=0,den=0;
	for(i=0;i<N;i++)
	{
		num+=nodes[i].keynbrsize;
		den+=nodes[i].phynbrsize;
	}
	printf("Average neighbourhood size = %lf\n",(double)den/(double)N );
	double DKEp = (double)((double)num/(double)den);

	printf("\nEG Scheme\nDistributing keys.... \n");
	printf("\nComputing key neighbourhood in direct key establishment phase....\n");

	printf("Simulated Network connectivity for direct key phase is %lf \n", DKEp);

	double up=0,down=0,res=1;
	for(i=0;i<KEYRING_SIZE;i++)
	{
		up = KEYPOOL_SIZE - KEYRING_SIZE -i;
		down = KEYPOOL_SIZE - i;
		res *= (up/down);
	}
	printf("Theoritical connectivity = %lf\n",1-res);

	int i_phynbr,i_keynbr,temp_i,index,hop1=0,neighbour_index;
	// indirect key establishment phase
	printf("\nindirect key establishment phase\n");
/*	for(i=0;i<N;i++)
	{
		memset(hop2node,0,10000);
		//checking physical neighbours
		for(i_phynbr=0;i_phynbr<nodes[i].phynbrsize;i_phynbr++)
		{
			flag=0;
			for(i_keynbr=0;i_keynbr<nodes[i].keynbrsize;i_keynbr++)
			{
				if(nodes[i].phynbr[i_phynbr]==nodes[i].keynbr[i_keynbr])
					{flag=1;break;}
			}
			if(flag==0)
			{
				// physical neighbour but not key neighbour then search for
				// that node which is phy neighbour as well as key nbr
				index = nodes[i].phynbr[i_phynbr];
				//searching in phynbr list
				for(temp_i=0;temp_i<nodes[i].phynbrsize;temp_i++)
				{
					neighbour_index = nodes[i].phynbr[temp_i];
					if(isPresent(nodes[neighbour_index].phynbr,nodes[neighbour_index].phynbrsize,index) 
							&& isPresent(nodes[neighbour_index].keynbr,nodes[neighbour_index].keynbrsize,index)
							&& !hop2node[neighbour_index] )
						{hop1++;hop2node[neighbour_index]=1;break; }
				}
				
			}
		}
	}*/

	int hop2=0,index1,index2,index3,i1,i2,i3;
	for(i=0;i<N;i++)
	{
		memset(hop2node,0,10000);
		for(i1=0;i1<nodes[i].phynbrsize;i1++)
		{
			index1 = nodes[i].phynbr[i1];
			if(isPresent(nodes[i].keynbr,nodes[i].keynbrsize,index1))
			{
				for(i2=0;i2<nodes[index1].phynbrsize;i2++)
				{
					index2 = nodes[index1].phynbr[i2];
					if(isPresent(nodes[index1].keynbr,nodes[index1].keynbrsize,index2))
					{
						// 2nd hop reaches to node
						// check whether it is phy neighbour but not key nbr of ith
						if(isPresent(nodes[i].phynbr,nodes[i].phynbrsize,index2) 
							&& !isPresent(nodes[i].keynbr,nodes[i].keynbrsize,index2)
							&& !hop2node[index2])
							{hop1++;hop2node[index2]=1;}
					}
				}
			}
		}
	}


	double IKEp1 = 1 - (1 - DKEp)*pow((1 - DKEp*DKEp),avgdist);
	printf("Theoritical Network connectivity for 1 hop is %lf\n", IKEp1);

	double hop1ans =  ((double)hop1 + (double)num)/(double)den;
	printf("Practical Network connectivity for 1 hop is %lf\n", 2* hop1ans );



	for(i=0;i<N;i++)
	{
		memset(hop2node,0,10000);
		for(i1=0;i1<nodes[i].phynbrsize;i1++)
		{
			index1 = nodes[i].phynbr[i1];
			if(isPresent(nodes[i].keynbr,nodes[i].keynbrsize,index1))
			{
				for(i2=0;i2<nodes[index1].phynbrsize;i2++)
				{
					index2 = nodes[index1].phynbr[i2];
					if(isPresent(nodes[index1].keynbr,nodes[index1].keynbrsize,index2))
					{
						for(i3=0;i3<nodes[index2].phynbrsize;i3++)
						{
							index3 = nodes[index2].phynbr[i3];
							if(isPresent(nodes[index2].keynbr,nodes[index2].keynbrsize,index3))
							{
								// 3rd hop reaches to node
								// check whether it is phy neighbour but not key nbr of ith
								if(isPresent(nodes[i].phynbr,nodes[i].phynbrsize,index3) 
									&& !isPresent(nodes[i].keynbr,nodes[i].keynbrsize,index3)
									&& !hop2node[index3])
									{hop2++;hop2node[index3]=1;}
							}
						}
					}
				}
			}
		}
	}


	double IKEp2 = 1 - (1 - IKEp1)*pow((1 - DKEp*IKEp1),avgdist);
	printf("\nTheoritical Network connectivity for 2 hops is %lf\n", IKEp2 );

	printf("Practical Network connectivity for 2 hops is %lf\n", 2*hop1ans + ( (double)(hop2/2) /(double)den) );
	

return 0;
}

int isPresent(int array[],int size,int item)
{
	int i=0,flag=0;
	for(i=0;i<size;i++)
		if(array[i]==item)
		{
			flag=1;break;
		}
	return flag;
}