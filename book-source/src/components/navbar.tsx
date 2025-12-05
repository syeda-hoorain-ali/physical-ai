import Link from "@docusaurus/Link";
import { InteractiveHoverButton } from "./ui/interactive-hover-button";
import { BookOpen } from "lucide-react";

export const Navbar = () => {
  return (
    <nav className="fixed top-0 z-50 w-full border-b border-border/50 bg-background/80 backdrop-blur-xl">
      <div className="container mx-auto flex h-16 items-center justify-between px-4">
        <div className="flex items-center gap-2">
          <BookOpen className="h-6 w-6 text-primary" />
          <span className="font-bold text-lg text-foreground">Physical AI Textbook</span>
        </div>

        <Link to="/docs/introduction-to-physical-ai-and-embodied-intelligence">
          <InteractiveHoverButton className="px-6 py-2.5 text-sm">
            Read Book
          </InteractiveHoverButton>
        </Link>
      </div>
    </nav>
  );
};
