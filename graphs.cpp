#include <bits/stdc++.h>
#include<vector>

using namespace std;
#define ll long long 
#define ld long double
#define vi vector<int>
#define vll vector<long long>
#define vvi vector<vector<int>>
#define vvll vector<vector<long long>>
#define nl "\n"
#define yes cout <<"YES" << nl
#define no cout << "NO" << nl
#define PI 3.141592653589793238
#define pb push_back
#define puf push_front
#define pu pop_back
#define pof pop_front
#define f(a,b,c) for(int a=b; a<c ; a++)

const int MOD=1E9+7;

 void dfs(vector<int>adj[],vector<int>&fin , vector<int>&vis,int src,int &time){
    vis[src]=1;
    time++;
    for(int x : adj[src]){
        if(vis[x]==0){
            dfs(adj,fin,vis,x,time);
            time++;
        }
    }
    fin[src]=time;
    return;
}
vector<int> topologicalSort(vector<int>adj[],int nodes,vector<int>&vis) {
    int n=nodes;
    vector<int>fin(n,0);
    int time=0;
    for(int i=0 ; i<nodes ; i++){
        if(vis[i]==0){
            dfs(adj,fin,vis,i,time);
        }
    }
    vector<pair<int,int>>temp;
    for(int i=0 ; i<n; i++){
        pair<int,int>x={fin[i],i};
        temp.push_back(x);
    }
    sort(temp.begin(),temp.end());
    vector<int>ans;
    for(int i=n-1 ; i>=0;  i--){
        ans.push_back(temp[i].second);
    }
    return ans;
}




//using priority queue
vector <int> dijkstra_queue(int V, vector<vector<int>> adj[], int S)
    {
        int n=V;
        priority_queue<pair<int,int>,vector<pair<int,int>>,greater<pair<int,int>>>q;
        vector<int>dis(n,1e9);
        dis[S]=0;
        q.push({0,S});
        while(!q.empty()){
            int dist=q.top().first;
            int u=q.top().second;
            q.pop();
            for(auto x : adj[u]){
                int w=x[1];
                int v=x[0];
                if(dist+w<dis[v]){
                    dis[v]=dist+w;
                    q.push({dis[v],v});
                }
            }
        }
        return dis;
        
    }

       //using sets;
     vector <int> dijkstra_sets(int V, vector<vector<int>> adj[], int S)
    {
        int n=V;
        set<pair<int,int>>st;
        vector<int>dis(n,1e9);
        dis[S]=0;
        st.insert({0,S});
        while(!st.empty()){
            auto it= *(st.begin());
            int dist=it.first;
            int u=it.second;
            st.erase(it);
            
            for(auto x : adj[u]){
                int w=x[1];
                int v=x[0];
                if(dist+w<dis[v]){
                    if(dis[v]!=1e9){
                        st.erase({dis[v],v});
                    }
                    dis[v]=dist+w;
                    st.insert({dis[v],v});
                }
            }
        }
        return dis;
        
    }
     vector <int> dijkstra_path(int V, vector<vector<int>> adj[], int S,int dest,vector<int>&parent)
    {
        int n=V;
        set<pair<int,int>>st;
        vector<int>dis(n,1e9);
        dis[S]=0;
        parent[S]=S;
        st.insert({0,S});
        while(!st.empty()){
            auto it= *(st.begin());
            int dist=it.first;
            int u=it.second;
            st.erase(it);
            for(auto x : adj[u]){
                int w=x[1];
                int v=x[0];
                if(dist+w<dis[v]){
                    if(dis[v]!=1e9){
                        st.erase({dis[v],v});
                    }
                    parent[v]=u;
                    dis[v]=dist+w;
                    st.insert({dis[v],v});
                }
            }
        }
        vector<int>path;
        while(parent[dest]!=dest){
            path.push_back(dest);
            if(parent[dest]==-1){
                vector<int>temp;
                temp.push_back(-1);
                return temp;
            }
            dest=parent[dest];
        }
        path.push_back(dest);
        reverse(path.begin(),path.end());
        return path;
    }

	vector<vector<int>>floyd_warshall(vector<vector<int>>&matrix){
	    int n=matrix.size();
         vector<vector<int>>parent(n,vector<int>(n,-1));
	    for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
                if(matrix[i][j]!=-1 && i!=j){
                    parent[i][j]=i;
                }
				if (matrix[i][j] == -1) {
					matrix[i][j] = 1e9;
				}
				if (i == j) matrix[i][j] = 0;
			}
		}
	    vector<vector<int>>prev=matrix;
	    vector<vector<int>>cur=matrix;
	    for(int k=0 ; k<n ; k++){
	        for(int i=0 ;i<n;  i++){
	            for(int j=0 ; j<n ; j++){
	                cur[i][j]=min(prev[i][j],prev[i][k]+prev[k][j]);
                    if(prev[i][k]+prev[k][j]<prev[i][j]){
                        parent[i][j]=k;
                    }
	            }
	        }
	        prev=cur;
	    }
         matrix=cur;
	    for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				if (matrix[i][j] == 1e9) {
					matrix[i][j] = -1;
				}
			}
		}
        return matrix;
	}

    vector<int> bellman_ford(int V, vector<vector<int>>& edges, int S) {
        //only for directed graph. if there is undirected graph convert it to directide by splitting every edge into two with same weight;
        int n=V;
        vector<int>dist(n,1e8);
        dist[S]=0;
        
        for(int i=0 ;i<n-1 ;i++){
            for(auto x: edges){
                int u=x[0];
                int v=x[1];
                int w=x[2];
                if( dist[u]!=1e8 && dist[u]+w<dist[v]){
                    dist[v]=dist[u]+w;
                }
            }
        }
        
        for(auto x: edges){
            int u=x[0];
            int v=x[1];
            int w=x[2];
            if(dist[u]!=1e8 && dist[u]+w<dist[v]){
                return {-1};
            }
        }
        
        return dist;
        
    }

    typedef struct triple {
         int first;
         int second;
         int third;
     
         bool operator>(const triple& other) const {
             return first > other.first;
         }
     
         bool operator>=(const triple& other) const {
             return first >= other.first;
         }
     
         bool operator<(const triple& other) const {
             return first < other.first;
         }
     
         bool operator<=(const triple& other) const {
             return first <= other.first;
         }
     }triple;


    int spanningTree_prims(int V, vector<vector<int>> adj[])
    {
        priority_queue<pair<int,int>,vector<pair<int,int>>,greater<pair<int,int>>>q;
        q.push({0,0});
        int sum=0;
        vector<int>vis(V,0);
        while(!q.empty()){
            int w=q.top().first;
            int node=q.top().second;
            q.pop();
            if(vis[node]==1) continue;
            vis[node]=1;
            sum+=w;
            for(auto x: adj[node]){
                int an=x[0];
                int wt=x[1];
                if(vis[an]==0){
                    q.push({wt,an});
                }
            }
        }
        return sum;
    }

     class DisjointSet{
        public: 
        vector<int>rank,parent,size;
        DisjointSet(int n){
            rank.resize(n+1,0);
            size.resize(n+1);
            parent.resize(n+1);

            for(int i=0 ; i<=n ; i++){
                parent[i]=i;
                size[i]=1;
            }
        }

        int find_up(int node){
            if(node==parent[node]) return node;
            return parent[node]=find_up(parent[node]);
        }

        void unionByRank(int u,int v){
            int ult_u=find_up(u);
            int ult_v=find_up(v);
            if(ult_u==ult_v) return;
            if(rank[ult_u]<rank[ult_v]){
                parent[ult_u]=  ult_v;
            }
            else if(rank[ult_u]>rank[ult_v]){
                parent[ult_v]=ult_u;
            }
            else{
                parent[ult_v]=ult_u;
                rank[ult_u]++;
            }
        }
        void unionBySize(int u,int v){
            int ult_u=find_up(u);
            int ult_v=find_up(v);
            if(ult_u==ult_v) return;
            if(size[ult_u]<size[ult_v]){
                parent[ult_u]=ult_v;
                size[ult_v]+=size[ult_u];
            }
            else{
                parent[ult_v]=ult_u;
                size[ult_u]+=size[ult_v];
            }
        }
     };

int kruskal(int V, vector<vector<int>> adj[]){
        vector<pair<int,pair<int,int>>>edges;
        for(int i=0 ;i<V ; i++){
            for(auto x: adj[i]){
                int an= x[0];
                int wt=x[1];
                int node =i;
                edges.push_back({wt,{node,an}});
            }
        }
        DisjointSet ds(V);
        sort(edges.begin(),edges.end());
        int ans=0;
        for(auto x: edges){
            int wt =x.first;
            int u=x.second.first;
            int v=x.second.second;
            if(ds.find_up(u)!=ds.find_up(v)){
                ans+=wt;
                ds.unionBySize(u,v);
            }
        }
        return ans;
}


int main() {
    DisjointSet ds(7);
    ds.unionBySize(1,2);
    ds.unionBySize(2,3);
    ds.unionBySize(4,5);
    ds.unionBySize(6,7);
    ds.unionBySize(5,6);
    if(ds.find_up(3)==ds.find_up(7)){
        cout<<"same"<<endl;
    }
    else{
        cout<<"not same"<<endl;
    }
    ds.unionBySize(3,7);
    if(ds.find_up(3)==ds.find_up(7)){
        cout<<"same"<<endl;
    }
    else{
        cout<<"not same"<<endl;
    }
	return 0;
}
