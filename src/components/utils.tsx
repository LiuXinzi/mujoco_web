export interface ZoomSliderProps {
  value: number;            
  onChange: (v: number) => void; 
  min?: number;              
  max?: number;               
  step?: number;              
}

export default function ZoomSlider({
  value,
  onChange,
  min = 1,
  max = 5,
  step = 0.1,
}: ZoomSliderProps) {
  return (
    <input
      type="range"
      min={min}
      max={max}
      step={step}
      value={value}
      onChange={(e) => onChange(parseFloat(e.target.value))}
      style={{
        position: "absolute",
        bottom: 24,
        left: "50%",
        transform: "translateX(-50%)",
        width: 260,
      }}
      aria-label="camera distance"
    />
  );
}
