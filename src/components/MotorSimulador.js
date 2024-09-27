import React, { useState, useEffect, useRef } from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend } from 'recharts';

const MotorSimulator = () => {
  // Parâmetros do motor DC
  const J = 0.01;
  const b = 0.1;
  const K = 0.01;
  const R = 1;
  const L = 0.5;

  // Estados para os ganhos do PID e outros parâmetros
  const [kp, setKp] = useState(60.54);
  const [ki, setKi] = useState(79.8022);
  const [kd, setKd] = useState(45.4818);
  const [angle, setAngle] = useState(0);
  const [targetAngle, setTargetAngle] = useState(1); // Entrada degrau unitário
  const [disturbance, setDisturbance] = useState(0);
  const [data, setData] = useState([]);
  const [debugData, setDebugData] = useState([]);
  const [simulationMode, setSimulationMode] = useState('continuous');
  const [isSimulating, setIsSimulating] = useState(false);
  const [logOutput, setLogOutput] = useState('');

  // Refs para cálculos
  const velocityRef = useRef(0);
  const currentRef = useRef(0);
  const lastErrorRef = useRef(0);
  const integralErrorRef = useRef(0);
  const timeRef = useRef(0);

  const simulationStep = 0.01; // 10ms, como no modelo MATLAB

  const runMatlabSimulation = () => {
    setIsSimulating(true);
    let simulationData = [];
    let simulationDebugData = [];
    let currentAngle = 0;
    let currentVelocity = 0;
    let currentCurrent = 0;
    let lastError = 0;
    let integralError = 0;

    const simulationDuration = 100; // 100 segundos, como no modelo MATLAB
    const steps = simulationDuration / simulationStep;

    for (let i = 0; i < steps; i++) {
      const time = i * simulationStep;
      const error = targetAngle - currentAngle + disturbance;
      
      integralError += error * simulationStep;
      const derivativeError = (error - lastError) / simulationStep;
      const pidOutput = kp * error + ki * integralError + kd * derivativeError;

      const torque = K * currentCurrent;
      const backEmf = K * currentVelocity;
      const acceleration = (torque - b * currentVelocity) / J;
      currentVelocity += acceleration * simulationStep;
      const voltageInput = pidOutput;
      currentCurrent += ((voltageInput - R * currentCurrent - backEmf) / L) * simulationStep;

      currentAngle += currentVelocity * simulationStep;

      simulationData.push({ time, angle: currentAngle, target: targetAngle });
      simulationDebugData.push({
        time,
        angle: currentAngle,
        target: targetAngle,
        error,
        integralError,
        derivativeError,
        pidOutput,
        current: currentCurrent,
        velocity: currentVelocity
      });

      lastError = error;
    }

    setData(simulationData);
    setDebugData(simulationDebugData);

    // Análise de desempenho
    const settlingTime = simulationData.findIndex(d => Math.abs(d.angle - targetAngle) <= 0.02 * targetAngle);
    const riseTime = simulationData.findIndex(d => d.angle >= 0.9 * targetAngle);
    const peakValue = Math.max(...simulationData.map(d => d.angle));
    const overshoot = ((peakValue - targetAngle) / targetAngle) * 100;
    const steadyStateError = Math.abs(simulationData[simulationData.length - 1].angle - targetAngle);

    const log = `
Tempo de subida: ${(riseTime * simulationStep).toFixed(2)} s
Sobressinal: ${overshoot.toFixed(2)}%
Tempo de acomodação: ${(settlingTime * simulationStep).toFixed(2)} s
O sistema é estável.
Erro em regime permanente: ${(steadyStateError * 100).toFixed(2)}%
    `;

    setLogOutput(log);
    setAngle(currentAngle);
    setIsSimulating(false);
  };

  useEffect(() => {
    let interval;
    if (simulationMode === 'continuous' && isSimulating) {
      interval = setInterval(() => {
        timeRef.current += simulationStep;

        const error = targetAngle - angle + disturbance;
        
        integralErrorRef.current += error * simulationStep;
        const derivativeError = (error - lastErrorRef.current) / simulationStep;
        const pidOutput = kp * error + ki * integralErrorRef.current + kd * derivativeError;

        const torque = K * currentRef.current;
        const backEmf = K * velocityRef.current;
        const acceleration = (torque - b * velocityRef.current) / J;
        velocityRef.current += acceleration * simulationStep;
        const voltageInput = pidOutput;
        currentRef.current += ((voltageInput - R * currentRef.current - backEmf) / L) * simulationStep;

        setAngle(prevAngle => {
          const newAngle = prevAngle + velocityRef.current * simulationStep;
          const newData = { 
            time: timeRef.current, 
            angle: newAngle, 
            target: targetAngle 
          };
          setData(prevData => [...prevData.slice(-100), newData]);
          setDebugData(prevDebug => [...prevDebug, {
            ...newData,
            error,
            integralError: integralErrorRef.current,
            derivativeError,
            pidOutput,
            current: currentRef.current,
            velocity: velocityRef.current
          }]);
          return newAngle;
        });

        lastErrorRef.current = error;
      }, simulationStep * 1000);
    }

    return () => clearInterval(interval);
  }, [angle, targetAngle, kp, ki, kd, disturbance, simulationMode, isSimulating]);

  const rotorStyle = {
    width: '100px',
    height: '10px',
    backgroundColor: 'blue',
    position: 'absolute',
    top: '95px',
    left: '100px',
    transformOrigin: 'left center',
    transform: `rotate(${angle}deg)`,
    transition: 'transform 0.01s linear',
  };

  const downloadDebugData = () => {
    const csvContent = "data:text/csv;charset=utf-8," 
      + "Time,Angle,Target,Error,IntegralError,DerivativeError,PIDOutput,Current,Velocity\n"
      + debugData.map(row => Object.values(row).join(",")).join("\n");
    const encodedUri = encodeURI(csvContent);
    const link = document.createElement("a");
    link.setAttribute("href", encodedUri);
    link.setAttribute("download", "debug_data.csv");
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
  };

  return (
    <div style={{ padding: '20px' }}>
      <h2 style={{ fontSize: '24px', fontWeight: 'bold', marginBottom: '16px' }}>Simulador de Motor DC com Controle PID</h2>
      <div style={{ display: 'flex', gap: '16px', marginBottom: '16px' }}>
        <div>
          <label style={{ display: 'block' }}>Kp: </label>
          <input type="number" value={kp} onChange={(e) => setKp(Number(e.target.value))} style={{ border: '1px solid #ccc', padding: '4px' }} />
        </div>
        <div>
          <label style={{ display: 'block' }}>Ki: </label>
          <input type="number" value={ki} onChange={(e) => setKi(Number(e.target.value))} style={{ border: '1px solid #ccc', padding: '4px' }} />
        </div>
        <div>
          <label style={{ display: 'block' }}>Kd: </label>
          <input type="number" value={kd} onChange={(e) => setKd(Number(e.target.value))} style={{ border: '1px solid #ccc', padding: '4px' }} />
        </div>
      </div>
      <div style={{ display: 'flex', gap: '16px', marginBottom: '16px' }}>
        <div>
          <label style={{ display: 'block' }}>Ângulo Alvo: </label>
          <input type="number" value={targetAngle} onChange={(e) => setTargetAngle(Number(e.target.value))} style={{ border: '1px solid #ccc', padding: '4px' }} />
        </div>
        <div>
          <label style={{ display: 'block' }}>Perturbação: </label>
          <input type="number" value={disturbance} onChange={(e) => setDisturbance(Number(e.target.value))} style={{ border: '1px solid #ccc', padding: '4px' }} />
        </div>
      </div>
      <div style={{ marginBottom: '16px' }}>
        <label style={{ marginRight: '8px' }}>Modo de Simulação: </label>
        <select value={simulationMode} onChange={(e) => setSimulationMode(e.target.value)} style={{ padding: '4px' }}>
          <option value="continuous">Contínuo</option>
          <option value="matlab">MATLAB</option>
        </select>
      </div>
      <button onClick={() => simulationMode === 'matlab' ? runMatlabSimulation() : setIsSimulating(!isSimulating)} style={{ marginBottom: '16px', padding: '8px 16px', backgroundColor: '#4CAF50', color: 'white', border: 'none', cursor: 'pointer' }}>
        {simulationMode === 'matlab' ? 'Executar Simulação MATLAB' : (isSimulating ? 'Parar Simulação' : 'Iniciar Simulação')}
      </button>
      <div style={{ position: 'relative', width: '300px', height: '200px', border: '1px solid #ccc', marginBottom: '16px' }}>
        <div style={rotorStyle}></div>
      </div>
      <LineChart width={600} height={300} data={data}>
        <CartesianGrid strokeDasharray="3 3" />
        <XAxis dataKey="time" label={{ value: 'Tempo (s)', position: 'insideBottomRight', offset: -10 }} />
        <YAxis label={{ value: 'Ângulo (graus)', angle: -90, position: 'insideLeft' }} />
        <Tooltip />
        <Legend />
        <Line type="monotone" dataKey="angle" stroke="#8884d8" dot={false} />
        <Line type="monotone" dataKey="target" stroke="#82ca9d" dot={false} />
      </LineChart>
      <button onClick={downloadDebugData} style={{ marginTop: '16px', padding: '8px 16px', backgroundColor: '#4CAF50', color: 'white', border: 'none', cursor: 'pointer' }}>
        Download Debug Data
      </button>
      <div style={{ marginTop: '16px', whiteSpace: 'pre-wrap', fontFamily: 'monospace' }}>
        <h3>Log de Saída:</h3>
        {logOutput}
      </div>
    </div>
  );
};

export default MotorSimulator;