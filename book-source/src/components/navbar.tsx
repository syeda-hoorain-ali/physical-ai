import Link from "@docusaurus/Link";
import { InteractiveHoverButton } from "./ui/interactive-hover-button";
import { BookOpen, LogInIcon, UserIcon } from "lucide-react";
import { Button } from "./ui/button";
import { useAuth } from "../features/auth/hooks";

export const Navbar = () => {
  const { user, loading } = useAuth();

  return (
    <nav className="fixed top-0 z-50 w-full border-b border-border/50 bg-background/80 backdrop-blur-xl">
      <div className="container mx-auto flex h-16 items-center justify-between px-4">
        <div className="flex items-center gap-2">
          <BookOpen className="h-6 w-6 text-primary" />
          <span className="font-bold text-lg text-foreground">Physical AI Textbook</span>
        </div>

        <div className="flex items-center gap-3">
          {!loading && (
            user ? (
              <Link to="/profile">
                <Button variant="ghost" size="sm" className="gap-2">
                  <UserIcon className="h-4 w-4" />
                  <span className="hidden sm:inline">Profile</span>
                </Button>
              </Link>
            ) : (
              <Link to="/auth">
                <Button variant="ghost" size="sm" className="gap-2">
                  <LogInIcon className="h-4 w-4" />
                  <span className="hidden sm:inline">Sign In</span>
                </Button>
              </Link>
            )
          )}
          <Link to="/docs/introduction-to-physical-ai-and-embodied-intelligence">
            <InteractiveHoverButton className="px-6 py-2.5 text-sm">
              Read Book
            </InteractiveHoverButton>
          </Link>
        </div>
      </div>
    </nav>
  );
};
