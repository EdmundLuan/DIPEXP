# FFT Notes

## Lemma 1

-   Let $\omega_n^k = e^{-j2k\pi/n}$  ,  and then  
    $$
    \omega_n^{k+n/2} = -\omega^k_n
    $$

## Lemma 2
-	By definition in lemma 1, 
	$$
	\omega_n^{2k} = \omega^k_{n/2}
	$$

## Lemma 3

-   By the same definition, 
    $$
    \omega_n^{2k} = \omega_n^{2k+n}
    $$
    

## DFT

$$
\begin{equation}
\begin{aligned}
\mathscr F[f(x)] = F(u) = y_u &= \sum_{k=0}^{n-1}f(x_k)e^{-j2ku\pi/n}\\
&=\sum_{k=0}^{n-1}a_k \omega^{uk}_n\\
&{\rm where}\ u=0,1,\cdots,n-1
\end{aligned}
\end{equation}
$$

## FFT

-   Assume that $n=2^p$ where $p$ is an integer, for we can simply let $a_i=0$ if $i$ is larger that the actual $n$ that we need. 

-   Divide $y_u$ into two parts, with even-indexed and odd-indexed coefficients respectively:  
    $$
    \begin{equation}
    \begin{aligned}
    y_u &= \sum_{m=0}^{n/2-1}a_{2m}\omega_n^{2mu}+\sum_{m=0}^{n/2-1}a_{2m+1}\omega_n^{(2m+1)u}\\
    	&= \sum_{m=0}^{n/2-1}a_{2m}\omega_n^{2mu}+\omega_n^u \sum_{m=0}^{n/2-1}a_{2m+1}\omega_n^{2mu}
    \end{aligned}
    \end{equation}
    $$
    

-   According to lemma 2, it can be turned into:  
    $$
    y_u = \sum_{m=0}^{n/2-1}a_{2m}\omega_{n/2}^{mu}+\omega_n^u \sum_{m=0}^{n/2-1}a_{2m+1}\omega_{n/2}^{mu}\\
    {\rm where}\ u=0,1,\cdots,n/2-1
    $$
    

-   By then, the original problem has been divided into two subproblems with only half size. The two summation terms are of the same form as equation (4).  

-   As for the other half part of $y_u$ — where $u=n/2,n/2+1,\cdots,n$ , we can produce it in the following way:  
    $$
    \begin{equation}
    \begin{aligned}
    y_{u+n/2} &= \sum_{m=0}^{n/2-1}a_{2m}\omega_n^{2m(u+n/2)}+\omega_n^{u+(n/2)} \sum_{m=0}^{n/2-1}a_{2m+1}\omega_n^{2m(u+n/2)}\\
    		&= \sum_{m=0}^{n/2-1}a_{2m}\omega_n^{2mu+n}+\omega_n^{u+(n/2)} \sum_{m=0}^{n/2-1}a_{2m+1}\omega_n^{2mu+n}\\
    		&= \sum_{m=0}^{n/2-1}a_{2m}\omega_n^{2mu}+\omega_n^{u+(n/2)} \sum_{m=0}^{n/2-1}a_{2m+1}\omega_n^{2mu}&({\rm by\ lemma\ 3})\\
    		&= \sum_{m=0}^{n/2-1}a_{2m}\omega_n^{2mu}-\omega_n^{u} \sum_{m=0}^{n/2-1}a_{2m+1}\omega_n^{2mu}&({\rm by\ lemma\ 1})\\
    		&{\rm where}\ u=0, 1, \cdots, n/2-1
    \end{aligned}
    \end{equation}
    $$
    

## Description of Recursive FFT Algorithm

```c++
std::vector<complex> FFT(std::vector<double> a){
    std::vector<double> aOdd, aEven;
    std::vector<complex> yOdd, yEven, y;
    n = a.length();	// Need to ensure that n is a power of 2 before using this function.
    if(n==1)
        return a;
    wn=power(e, 2*PI*j/n);		// Omega_n
    w=1;
    for(int i=0;i<n/2;++i){
        aOdd[i] = a[2*i+1];		// Odd-indexed coefficients
        aEven[i] = a[2*i];		// Even-indexed coefficients
    }
    yOdd = FFT(aOdd);
    yEven = FFT(aEven);
    for(int u=0; u<n/2; ++u){
        y[u] = yEven[u] + w*yOdd[u];
        y[u+n/2] = yEven[u] - w*yOdd[u];
        w = w*wn;
    }
    return y;
}

```

