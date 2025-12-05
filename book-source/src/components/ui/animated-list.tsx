import { cn } from "@site/src/lib/utils";
import { useEffect, useState } from "react";

interface AnimatedListProps {
  items: React.ReactNode[];
  className?: string;
  delay?: number;
}

export const AnimatedList = ({
  items,
  className,
  delay = 200,
}: AnimatedListProps) => {
  const [visibleItems, setVisibleItems] = useState<number[]>([]);

  useEffect(() => {
    const timeouts: NodeJS.Timeout[] = [];
    
    items.forEach((_, index) => {
      const timeout = setTimeout(() => {
        setVisibleItems((prev) => [...prev, index]);
      }, index * delay);
      timeouts.push(timeout);
    });

    return () => {
      timeouts.forEach(clearTimeout);
    };
  }, [items, delay]);

  return (
    <ul className={cn("space-y-4", className)}>
      {items.map((item, index) => (
        <li
          key={index}
          className={cn(
            "opacity-0 transition-all duration-500",
            visibleItems.includes(index) && "animate-slide-in-up opacity-100"
          )}
          style={{
            animationDelay: `${index * 0.1}s`,
          }}
        >
          {item}
        </li>
      ))}
    </ul>
  );
};
