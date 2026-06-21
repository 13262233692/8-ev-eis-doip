package battery

import (
	"math"
	"sync"
)

type EISPoint struct {
	Frequency float64
	ZReal     float64
	ZImag     float64
}

type RandlesParams struct {
	Rs   float64
	Rct  float64
	Cdl  float64
	W    float64
	Tau  float64
}

type FittingResult struct {
	Params     RandlesParams
	Residual   float64
	Iterations int
	Converged  bool
}

type TrustRegionConfig struct {
	MaxIter    int
	Tol        float64
	InitialRadius float64
	MaxRadius  float64
	Eta1       float64
	Eta2       float64
	Gamma1     float64
	Gamma2     float64
	Gamma3     float64
}

func DefaultTrustRegionConfig() TrustRegionConfig {
	return TrustRegionConfig{
		MaxIter:       200,
		Tol:           1e-10,
		InitialRadius: 1.0,
		MaxRadius:     100.0,
		Eta1:          0.05,
		Eta2:          0.9,
		Gamma1:        0.5,
		Gamma2:        2.0,
		Gamma3:        0.25,
	}
}

func RandlesImpedance(params RandlesParams, omega float64) (float64, float64) {
	rs := params.Rs
	rct := params.Rct
	cdl := params.Cdl
	w := params.W
	tau := params.Tau

	if tau <= 0 {
		tau = 1e-6
	}

	sqrtOmega := math.Sqrt(omega)
	wReal := w / sqrtOmega
	wImag := -w / sqrtOmega

	cdlImag := omega * cdl
	if cdlImag < 1e-16 {
		cdlImag = 1e-16
	}
	zCdlReal := 0.0
	zCdlImag := -1.0 / cdlImag

	warburgReal := wReal
	warburgImag := wImag

	zParReal := rct + warburgReal
	zParImag := warburgImag

	denom := zCdlImag*zCdlImag + zCdlReal*zCdlReal
	if math.Abs(denom) < 1e-16 {
		denom = 1e-16
	}
	yCdlReal := zCdlReal / denom
	yCdlImag := -zCdlImag / denom

	denom2 := zParReal*zParReal + zParImag*zParImag
	if math.Abs(denom2) < 1e-16 {
		denom2 = 1e-16
	}
	yParReal := zParReal / denom2
	yParImag := -zParImag / denom2

	yTotalReal := yCdlReal + yParReal
	yTotalImag := yCdlImag + yParImag

	yDenom := yTotalReal*yTotalReal + yTotalImag*yTotalImag
	if math.Abs(yDenom) < 1e-16 {
		yDenom = 1e-16
	}
	zParallelReal := yTotalReal / yDenom
	zParallelImag := -yTotalImag / yDenom

	zTotalReal := rs + zParallelReal
	zTotalImag := zParallelImag

	return zTotalReal, zTotalImag
}

func computeResiduals(params RandlesParams, data []EISPoint) []float64 {
	n := len(data)
	residuals := make([]float64, 2*n)

	for i, pt := range data {
		omega := 2.0 * math.Pi * pt.Frequency
		zr, zi := RandlesImpedance(params, omega)
		residuals[2*i] = zr - pt.ZReal
		residuals[2*i+1] = zi - pt.ZImag
	}

	return residuals
}

func computeCost(residuals []float64) float64 {
	var sum float64
	for _, r := range residuals {
		sum += r * r
	}
	return 0.5 * sum
}

func computeJacobian(params RandlesParams, data []EISPoint, step float64) [][]float64 {
	n := len(data)
	m := 5
	jac := make([][]float64, 2*n)
	for i := range jac {
		jac[i] = make([]float64, m)
	}

	baseResiduals := computeResiduals(params, data)

	paramSteps := []float64{
		math.Max(math.Abs(params.Rs)*step, step),
		math.Max(math.Abs(params.Rct)*step, step),
		math.Max(math.Abs(params.Cdl)*step, step),
		math.Max(math.Abs(params.W)*step, step),
		math.Max(math.Abs(params.Tau)*step, step),
	}

	for j := 0; j < m; j++ {
		p := params
		switch j {
		case 0:
			p.Rs += paramSteps[j]
		case 1:
			p.Rct += paramSteps[j]
		case 2:
			p.Cdl += paramSteps[j]
		case 3:
			p.W += paramSteps[j]
		case 4:
			p.Tau += paramSteps[j]
		}

		perturbedResiduals := computeResiduals(p, data)

		h := paramSteps[j]
		for i := 0; i < 2*n; i++ {
			jac[i][j] = (perturbedResiduals[i] - baseResiduals[i]) / h
		}
	}

	return jac
}

func matVecMul(A [][]float64, x []float64) []float64 {
	m := len(A)
	n := len(x)
	result := make([]float64, m)
	for i := 0; i < m; i++ {
		for j := 0; j < n; j++ {
			result[i] += A[i][j] * x[j]
		}
	}
	return result
}

func transposeMatVecMul(A [][]float64, x []float64) []float64 {
	m := len(A)
	n := len(A[0])
	result := make([]float64, n)
	for j := 0; j < n; j++ {
		for i := 0; i < m; i++ {
			result[j] += A[i][j] * x[i]
		}
	}
	return result
}

func dot(a, b []float64) float64 {
	var sum float64
	for i := range a {
		sum += a[i] * b[i]
	}
	return sum
}

func norm(v []float64) float64 {
	return math.Sqrt(dot(v, v))
}

func vecAdd(a, b []float64) []float64 {
	r := make([]float64, len(a))
	for i := range a {
		r[i] = a[i] + b[i]
	}
	return r
}

func vecSub(a, b []float64) []float64 {
	r := make([]float64, len(a))
	for i := range a {
		r[i] = a[i] - b[i]
	}
	return r
}

func vecScale(a []float64, s float64) []float64 {
	r := make([]float64, len(a))
	for i := range a {
		r[i] = a[i] * s
	}
	return r
}

func solveDogleg(J [][]float64, r []float64, delta float64) []float64 {
	m := len(J)
	n := len(J[0])

	Jtr := transposeMatVecMul(J, r)
	Jx := matVecMul(J, Jtr)
	gNorm2 := dot(Jtr, Jtr)
	JxNorm2 := dot(Jx, Jx)

	if JxNorm2 < 1e-16 {
		return vecScale(Jtr, delta/math.Max(norm(Jtr), 1e-16))
	}

	alpha := gNorm2 / JxNorm2
	pu := vecScale(Jtr, alpha)
	puNorm := norm(pu)

	if puNorm <= delta {
		JtJ := make([][]float64, n)
		for i := range JtJ {
			JtJ[i] = make([]float64, n)
		}
		for k := 0; k < m; k++ {
			for i := 0; i < n; i++ {
				for j := 0; j < n; j++ {
					JtJ[i][j] += J[k][i] * J[k][j]
				}
			}
		}

		pb := solveGaussNewton(JtJ, Jtr)
		pbNorm := norm(pb)

		if pbNorm <= delta {
			return pb
		}

		diff := vecSub(pb, pu)
		dotPuDiff := dot(pu, diff)
		diffNorm2 := dot(diff, diff)
		puNorm2 := puNorm * puNorm

		tau := 1.0
		aCoeff := diffNorm2
		bCoeff := 2.0 * dotPuDiff
		cCoeff := puNorm2 - delta*delta

		discriminant := bCoeff*bCoeff - 4.0*aCoeff*cCoeff
		if discriminant > 0 {
			sqrtDisc := math.Sqrt(discriminant)
			tau1 := (-bCoeff + sqrtDisc) / (2.0 * aCoeff)
			tau2 := (-bCoeff - sqrtDisc) / (2.0 * aCoeff)

			if tau1 >= 0 && tau1 <= 1 {
				tau = tau1
			} else if tau2 >= 0 && tau2 <= 1 {
				tau = tau2
			}
		}

		return vecAdd(pu, vecScale(diff, tau))
	}

	return vecScale(pu, delta/puNorm)
}

func solveGaussNewton(JtJ [][]float64, Jtr []float64) []float64 {
	n := len(JtJ)
	A := make([][]float64, n)
	for i := range A {
		A[i] = make([]float64, n+1)
		copy(A[i][:n], JtJ[i])
		A[i][n] = Jtr[i]
	}

	for i := 0; i < n; i++ {
		A[i][i] += 1e-8
	}

	for i := 0; i < n; i++ {
		maxRow := i
		maxVal := math.Abs(A[i][i])
		for k := i + 1; k < n; k++ {
			if math.Abs(A[k][i]) > maxVal {
				maxVal = math.Abs(A[k][i])
				maxRow = k
			}
		}
		A[i], A[maxRow] = A[maxRow], A[i]

		pivot := A[i][i]
		if math.Abs(pivot) < 1e-16 {
			continue
		}
		for j := i; j <= n; j++ {
			A[i][j] /= pivot
		}

		for k := 0; k < n; k++ {
			if k != i && math.Abs(A[k][i]) > 1e-16 {
				factor := A[k][i]
				for j := i; j <= n; j++ {
					A[k][j] -= factor * A[i][j]
				}
			}
		}
	}

	x := make([]float64, n)
	for i := 0; i < n; i++ {
		x[i] = A[i][n]
	}
	return x
}

func paramsToVec(p RandlesParams) []float64 {
	return []float64{p.Rs, p.Rct, p.Cdl, p.W, p.Tau}
}

func vecToParams(v []float64) RandlesParams {
	return RandlesParams{
		Rs:   math.Max(v[0], 1e-6),
		Rct:  math.Max(v[1], 1e-6),
		Cdl:  math.Max(v[2], 1e-9),
		W:    math.Max(v[3], 1e-6),
		Tau:  math.Max(v[4], 1e-6),
	}
}

func TrustRegionReflectiveFit(data []EISPoint, initial RandlesParams, config TrustRegionConfig) FittingResult {
	if len(data) < 3 {
		return FittingResult{Params: initial, Residual: math.Inf(1), Converged: false}
	}

	mu := sync.Mutex{}
	mu.Lock()
	defer mu.Unlock()

	if config.MaxIter <= 0 {
		config = DefaultTrustRegionConfig()
	}

	params := initial
	residuals := computeResiduals(params, data)
	cost := computeCost(residuals)

	radius := config.InitialRadius
	if radius <= 0 {
		radius = 1.0
	}

	result := FittingResult{
		Params:     params,
		Residual:   cost,
		Iterations: 0,
		Converged:  false,
	}

	for iter := 0; iter < config.MaxIter; iter++ {
		result.Iterations = iter

		jac := computeJacobian(params, data, 1e-6)

		step := solveDogleg(jac, residuals, radius)
		if norm(step) < 1e-16 {
			break
		}

		paramVec := paramsToVec(params)
		newParamVec := vecSub(paramVec, step)
		newParams := vecToParams(newParamVec)

		newResiduals := computeResiduals(newParams, data)
		newCost := computeCost(newResiduals)

		Jstep := matVecMul(jac, step)
		linearReduction := dot(step, transposeMatVecMul(jac, residuals)) - 0.5*dot(Jstep, Jstep)
		actualReduction := cost - newCost

		var rho float64
		if math.Abs(linearReduction) > 1e-16 {
			rho = actualReduction / linearReduction
		}

		if rho > config.Eta1 {
			params = newParams
			residuals = newResiduals
			cost = newCost
			result.Params = params
			result.Residual = cost
		}

		if rho < config.Eta1 {
			radius *= config.Gamma1
		} else if rho > config.Eta2 {
			radius = math.Min(config.Gamma2*radius, config.MaxRadius)
		}

		if math.Abs(actualReduction) < config.Tol || cost < config.Tol {
			result.Converged = true
			break
		}

		if radius < 1e-12 {
			break
		}
	}

	if result.Iterations >= config.MaxIter-1 {
		result.Converged = result.Residual < config.Tol*100
	}

	return result
}

func InitialGuessRandles(data []EISPoint) RandlesParams {
	if len(data) == 0 {
		return RandlesParams{Rs: 0.01, Rct: 0.1, Cdl: 1e-4, W: 0.01, Tau: 1.0}
	}

	var minReal, maxReal float64
	minReal = math.Inf(1)
	maxReal = math.Inf(-1)

	for _, pt := range data {
		if pt.ZReal < minReal {
			minReal = pt.ZReal
		}
		if pt.ZReal > maxReal {
			maxReal = pt.ZReal
		}
	}

	rs := math.Max(minReal*0.9, 1e-4)
	rct := math.Max(maxReal-minReal, 1e-3)
	cdl := 1e-4
	w := 0.05 * rct
	tau := 1.0

	if len(data) >= 2 {
		idx0 := 0
		idx1 := len(data) - 1
		f0 := data[idx0].Frequency
		f1 := data[idx1].Frequency
		if f0 > 0 && f1 > 0 {
			tau = 1.0 / (2.0 * math.Pi * math.Sqrt(f0*f1))
		}
	}

	return RandlesParams{
		Rs:   rs,
		Rct:  rct,
		Cdl:  cdl,
		W:    w,
		Tau:  tau,
	}
}

func ComputeSOH(params RandlesParams, reference RandlesParams) float64 {
	rctRatio := params.Rct / reference.Rct
	if rctRatio < 0.5 {
		rctRatio = 0.5
	}
	if rctRatio > 3.0 {
		rctRatio = 3.0
	}
	soh := 100.0 * (1.0 - (rctRatio-1.0)*0.4)
	if soh > 100 {
		soh = 100
	}
	if soh < 0 {
		soh = 0
	}
	return soh
}
