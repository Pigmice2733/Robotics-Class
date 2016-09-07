template <typename V, int N> class MovingAverage
{
public:
  
    MovingAverage(V def = 0) : sum(0), p(0)
    {
        for (int i = 0; i < N; i++) {
            samples[i] = def;
            sum += samples[i];
        }
    }
    
  
    V add(V new_sample)
    {
        sum = sum - samples[p] + new_sample;
        samples[p++] = new_sample;
        if (p >= N)
            p = 0;
        return sum / N;
    }
    
private:
    V samples[N];
    V sum;
    V p;
};
